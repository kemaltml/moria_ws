floor
   **base_joint**
        base
            **left_wheel_joint**
                left_wheel
            **right_wheel_joint**
                right_wheel
            **caster_joint**
                caster_link
            **legs_joint**
                legs
                    **body_joint**
                        body
                            **camera_joint**
                                camera
                                    *gazebo plugin*
                                    **camera_rgb_joint**
                                        camera_rgb_frame
                                        **camera_rgb_optical_joint**
                                            camera_rgb_optical_frame
                                    **camera_depth_joint**
                                        camera depth frame
                                            **camera_depth_optical_joint**
                                                camera_depth_optical_frame
            **imu_joint**
                imu
                    *gazebo plugin*
            **scan_joint_f**
                base_scan_f
                    *gazebo plugin*
            **scan_joint_b**
                base_scan_b
                    *gazebo plugin*


*moria diff drive* : libgazebo_ros_diff_drive.so
*moria joint state*: libgazebo_ros_joint_state_publisher.so
*laser_controller_front*: libgazebo_ros_ray_sensor.so
*laser_controller_back*: libgazebo_ros_ray_sensor.so
*camera_controller* : libgazebo_ros_camera.so
