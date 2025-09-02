'''
Neao Andor Camera Measurement
'''
from ScopeFoundry import Measurement
from ScopeFoundry.helper_funcs import sibling_path, load_qt_ui_file
from ScopeFoundry import h5_io
import pyqtgraph as pg
import numpy as np
import os, time
import matplotlib.pyplot as plt
import h5py


class SpimMeasure(Measurement):
    name = "SPIM_measure"

    def setup(self):
        self.ui_filename = sibling_path(__file__, "spim.ui")
        self.ui = load_qt_ui_file(self.ui_filename)

        self.settings.New('save_h5', dtype=bool, initial=False)

        self.settings.New('Z_step', dtype=int, unit='um', initial=10)
        self.settings.New('Z_series', dtype=int, initial=10)
        self.settings.New('Time_series', dtype=int, initial=1)

        self.settings.New('Measurement time', dtype=float, unit='s', initial=0)
        # self.settings.New('Acquisition time', dtype=float, unit='s', initial=0)

        # how often we want to update the display
        self.settings.New('refresh_period', dtype=float, unit='s', spinbox_decimals=3, initial=0.05, vmin=0)

        #RISOLUZIONE: dimensione del pixel della camera e sella profondità in z
        self.settings.New('xsampling', dtype=float, unit='um', initial=0.65)
        self.settings.New('ysampling', dtype=float, unit='um', initial=0.65)
        self.settings.New('zsampling', dtype=float, unit='um', initial=10)

        self.auto_range = self.settings.New('auto_range', dtype=bool, initial=True)
        self.settings.New('auto_levels', dtype=bool, initial=True)
        self.settings.New('level_min', dtype=int, initial=60)
        self.settings.New('level_max', dtype=int, initial=4000)

        '''AGGIUNGI UN BOTTONE:
        self.add_operation('name',op_func)
        example: self.add_operation('measure',self.measure)'''

        self.image_gen = self.app.hardware['NeoAndorHW']
        self.stage = self.app.hardware['PI_HW_NOT']
        self.shutter_measure = self.app.hardware['ShutterHW']

    def setup_figure(self):
        """
        Runs once during App initialization, after setup()
        This is the place to make all graphical interface initializations,
        build plots, etc.
        """
        # connect ui widgets to measurement/hardware settings or functions
        self.ui.start_pushButton.clicked.connect(self.start)
        self.ui.interrupt_pushButton.clicked.connect(self.interrupt)
        self.settings.save_h5.connect_to_widget(self.ui.save_h5_checkBox)
        self.settings.auto_levels.connect_to_widget(self.ui.autoLevels_checkbox)
        self.settings.auto_range.connect_to_widget(self.ui.autoRange_checkbox)
        self.settings.level_min.connect_to_widget(self.ui.min_doubleSpinBox)
        self.settings.level_max.connect_to_widget(self.ui.max_doubleSpinBox)

        # Set up pyqtgraph graph_layout in the UI
        self.imv = pg.ImageView()
        self.ui.imageLayout.addWidget(self.imv)
        colors = [(0, 0, 0),
                  (45, 5, 61),
                  (84, 42, 55),
                  (150, 87, 60),
                  (208, 171, 141),
                  (255, 255, 255)
                  ]
        cmap = pg.ColorMap(pos=np.linspace(0.0, 1.0, 6), color=colors)
        self.imv.setColorMap(cmap)

    def update_display(self):
        """
        Displays (plots) the numpy array self.buffer.
        This function runs repeatedly and automatically during the measurement run.
        its update frequency is defined by self.display_update_period
        """
        self.image_gen.read_from_hardware()
        self.stage.read_from_hardware()

        self.display_update_period = self.settings['refresh_period']

        # self.settings['progress'] = (self.frame_index + 1) * 100 / self.length
        self.settings['progress'] = self.frame_index * 100 / self.length

        if hasattr(self, 'img'):
            # show the image
            self.imv.setImage(self.img,               #self.img.T,
                              autoLevels=self.settings['auto_levels'],
                              autoRange=self.auto_range.val,
                              levelMode='mono'
                              )

            if self.settings['auto_levels']:
                lmin, lmax = self.imv.getHistogramWidget().getLevels()
                self.settings['level_min'] = lmin
                self.settings['level_max'] = lmax
            else:
                self.imv.setLevels(min=self.settings['level_min'],
                                   max=self.settings['level_max'])

    def measure(self):
        self.stage.read_from_hardware()
        self.image_gen.read_from_hardware()
        self.shutter_measure.read_from_hardware()

        self.stage.motor.go_home()
        self.stage.motor.wait_on_target()

        # print('translator start: ', self.stage.motor.get_position())

        step_length = self.settings['Z_step'] * 0.001
        self.length_saving = space_frame = self.settings['Z_series']
        time_frame = self.settings['Time_series']
        measure_time = self.settings['Measurement time']

        # if not self.speed_check(step_length):
        #     return

        start = 0
        stop = (step_length * space_frame)
        time_exp = self.image_gen.exposure_time.value
        print('start ', start)
        print('stop ', stop)
        print('time expo ', time_exp)
        # compute the velocity
        velocity = self.stage.motor.PI_velocity(time_exp, step_length)
        self.stage.motor.set_velocity(velocity)

        self.length = num_frame = space_frame * time_frame

        self.create_h5_file_ext(time_frame)

        self.image_gen.camera.acquisition_setup(num_frame)
        self.image_gen.camera.acquisition_start()

        self.frame_index = 0
        time_tot = np.zeros(shape=(time_frame, 1))
        self.shutter_measure.shutter.open_shutter()
        for time_idx in range(0, time_frame):
            t0 = time.perf_counter()
            # print('initial position: ', self.stage.motor.get_position())
            if time_idx % 2 == 0:
                self.stage.motor.move_absolute(stop)
            else:
                self.stage.motor.move_absolute(start)

            for frame_idx_ext in range(0, space_frame):
                t1 = time.perf_counter()
                self.frame_index += 1

                self.image_gen.camera.image_wait()
                self.img = self.image_gen.camera.image_read()

                # time_acq[frame_idx_ext,0] = time.perf_counter() - t1
                # t2 = time.perf_counter()
                # print('acquisition time: ', t_acqui)

                # access the group and save the image
                t_group = self.h5_group[f't{time_idx}']
                self.image_h5_ext = t_group['c0/image']
                self.image_h5_ext[frame_idx_ext, :, :] = self.img
                self.h5file.flush()

                # time_save[frame_idx_ext, 0] = time.perf_counter() - t1
                # print('timing: ', time_save[frame_idx_ext, 0])
                # time_acq[frame_idx_ext,1] = time.perf_counter() - t2
                # print('saving time: ', time.perf_counter() - t_a_1)

                if self.interrupt_measurement_called:
                    self.shutter_measure.shutter.close_shutter()
                    break

            time_tot[time_idx, 0] = time.perf_counter() - t0
            # print('time tot: ', time_tot)
            # print('measurement time: ', measure_time)

            # if measure_time > time_tot[time_idx, 0]:
            #     #chiudi lo shutter
            #     self.shutter_measure.shutter.close_shutter()
            #     #aspetta
            #     aspetta_frame = measure_time-time_tot[time_idx, 0]
            #     time.sleep(aspetta_frame)
            #     #apri lo shutter
            #     self.shutter_measure.shutter.open_shutter()

        x = np.arange(num_frame)
        # print(time_acq)

        # print('mean delay: ', np.mean(time_save))
        print('time tot: ', time_tot)
        print('mean single stacK: ', np.mean(time_tot))
        # print('mean camera time ', np.mean(time_acq[:,0]))
        # print('mean saving time', np.mean(time_acq[:,1]))
        self.image_gen.camera.acquisition_stop()
        self.shutter_measure.shutter.close_shutter()

        # creao la mip
        self.create_MIP(time_frame)

        # make sure to close the data file
        self.h5file.close()
        self.settings['save_h5'] = False
        # print('total acquisition time ', time.perf_counter()-t)

    def run(self):

        try:
            #start the camera
            self.frame_index = -1

            self.image_gen.read_from_hardware()
            self.length = self.image_gen.frame_num.val

            self.image_gen.camera.acquisition_clear()
            self.image_gen.camera.acquisition_setup(self.image_gen.frame_num.val)
            self.image_gen.camera.acquisition_start()

            # continuously get the last frame and put it in self.image, in order to
            # show it via self.update_display()

            while not self.interrupt_measurement_called:

                # If measurement is called, stop the acquisition, call self.measure
                # and get out of run()
                if self.settings['save_h5']:
                    # measure is triggered by save_h5 button
                    self.image_gen.camera.acquisition_stop()
                    self.measure()
                    break

                self.image_gen.camera.image_wait()
                self.img = self.image_gen.camera.image_read()

                if self.interrupt_measurement_called:
                    break

        finally:
            self.image_gen.camera.acquisition_stop()
            # self.image_gen.settings['trigger'] = 'int'
            if self.settings['save_h5'] and hasattr(self, 'h5file'):
                # make sure to close the data file
                self.h5file.close()
                self.settings['save_h5'] = False

    def create_saving_directory(self):

        if not os.path.isdir(self.app.settings['save_dir']):
            os.makedirs(self.app.settings['save_dir'])

    def create_h5_file_ext(self, t_frame):
        self.create_saving_directory()
        # file name creation
        timestamp = time.strftime("%y%m%d_%H%M%S", time.localtime())
        sample = self.app.settings['sample']
        # sample_name = f'{timestamp}_{self.name}_{sample}.h5'
        if sample == '':
            sample_name = '_'.join([timestamp, self.name])
        else:
            sample_name = '_'.join([timestamp, self.name, sample])
        #ho modificato
        self.fname = os.path.join(self.app.settings['save_dir'], sample_name + '.h5')

        self.h5file = h5_io.h5_base_file(app=self.app, measurement=self, fname=self.fname)
        self.h5_group = h5_io.h5_create_measurement_group(measurement=self, h5group=self.h5file)

        for t_idx in range (t_frame):
            # Group creation for each t_index
            t_group = self.h5_group.create_group(f't{t_idx}')

            img_size = (2160, 2560)
            dtype = 'uint16'

            length = self.length_saving

            self.image_h5_ext = t_group.create_dataset(name='c0/image',
                                                   shape=[length, img_size[0], img_size[1]],
                                                   dtype=dtype)
            self.image_h5_ext.attrs['element_size_um'] = [self.settings['zsampling'], self.settings['ysampling'],
                                                      self.settings['xsampling']]
            self.image_mip = t_group.create_dataset(name='c0/MIP',
                                                       shape=[img_size[0], img_size[1]],
                                                       dtype=dtype)
            self.image_mip.attrs['element_size_um'] = [self.settings['zsampling'], self.settings['ysampling'],
                                                      self.settings['xsampling']]

            self.positions_h5 = t_group.create_dataset(name='c0/position_mm',
                                                       shape=[length],
                                                       dtype=np.float32)

    def create_MIP(self, t_frame):
        hdf5_folder = self.app.settings['save_dir']
        hdf5_path = os.path.join(hdf5_folder, self.fname)

        dataset_name = 'c0/image'
        for time_idx in range(0, t_frame):
            group_name = f'/measurement/SPIM_measure/t{time_idx}'
            # print('greoup name ', group_name)
            with h5py.File(hdf5_path, 'r') as hf:
                # Load the image stack
                image_dataset = hf[f'{group_name}/{dataset_name}']
                image_stack = image_dataset[:]

            mip = np.max(image_stack, axis=0)
            t_group = self.h5_group[f't{time_idx}']
            self.image_mip = t_group['c0/MIP']
            self.image_mip[:, :] = mip
            self.h5file.flush()