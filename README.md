# MRS garmin LiDAR and Tarot gimbal LLCP driver
This nodelet utilizes [MRS LLCP protocol](https://github.com/ctu-mrs/mrs_llcp_ros) with arduino device to operate 2 garmin LiDAR Lite rangefinders and Tarot GoPro Hero 9 gimbal 

```mermaid
flowchart LR
A[mrs_garmin_and_gimbal_llcp] <-->|ROS messages| B[mrs_llcp_ros]
B <-->|UART| C[Arduino Nano]
C<-->|I2C|D[LiDAR up]
C<-->|I2C|E[LiDAR down]
C <-->|SBUS packets|F[Tarot gimbal]
```
# Requirements
* The arduino should have the [firmware](https://github.com/ctu-mrs/mrs_module_led_strip_driver/blob/master/firmware/firmware.ino) installed.
* The LLCP mesages are defined in the attached [header file](https://github.com/ctu-mrs/mrs_module_led_strip_driver/blob/master/firmware/msgs.h).
* Linked and compiled garmin_gimbal_llcp driver [nodelet](https://github.com/ctu-mrs/mrs_module_led_strip_driver) in your [workspace](https://ctu-mrs.github.io/docs/system/preparing_for_a_real-world_experiment.html#set-up-your-own-workspace).
* Linked and compiled MRS LLCP [nodelet](https://github.com/ctu-mrs/mrs_llcp) in modules workspace.
* Correctly set  `.rules` file in `/etc/udev/rules.d`, the LED module portname is `arduino`
  * For example, on a T650 drone, with the LED module in 2nd module slot, the `99-usb-serial_T650_PCB.rules` file includes this line:
  ```bash
   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", ENV{ID_USB_INTERFACE_NUM}=="02", SYMLINK+="arduino",OWNER="mrs",MODE="0666"
  ```
  * Of course, the `OWNER` and `ENV{ID_USB_INTERFACE_NUM}` parameters have to correspond with your setup
* Wire connection between the onboard computer and arduino device
