[app]
title = AVM Plotter
package.name = avmplotter
package.domain = org.avm.cnc
source.dir = .
source.include_exts = py,png,jpg,kv,atlas,json
version = 1.0
requirements = python3,kivy==2.3.0,pyserial,usb4a,usbserial4a,android
orientation = portrait
fullscreen = 0

android.permissions = android.permission.USB_PERMISSION,android.permission.READ_EXTERNAL_STORAGE,android.permission.WRITE_EXTERNAL_STORAGE
android.features = android.hardware.usb.host
android.api = 33
android.minapi = 21
android.ndk = 25b
android.accept_sdk_license = True

[buildozer]
log_level = 2
warn_on_root = 1
