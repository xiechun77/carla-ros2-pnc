import ctypes as C
import os
from PyQt5.QtGui import QImage

class XImage(C.Structure):
    _fields_ = [('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),('format',C.c_int),
        ('data',C.c_void_p),('byte_order',C.c_int),('bitmap_unit',C.c_int),('bitmap_bit_order',C.c_int),
        ('bitmap_pad',C.c_int),('depth',C.c_int),('bytes_per_line',C.c_int),('bits_per_pixel',C.c_int),
        ('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong)]
x=C.CDLL('libX11.so.6'); composite=C.CDLL('libXcomposite.so.1')
x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=C.c_void_p
d=x.XOpenDisplay(None)
if not d: raise SystemExit('No X display')
composite.XCompositeNameWindowPixmap.argtypes=[C.c_void_p,C.c_ulong]
composite.XCompositeNameWindowPixmap.restype=C.c_ulong
p=composite.XCompositeNameWindowPixmap(d,int(os.environ['RVIZ_WINDOW_ID'],16))
x.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int]
x.XGetImage.restype=C.POINTER(XImage)
i=x.XGetImage(d,p,0,0,int(os.environ['RVIZ_WIDTH']),int(os.environ['RVIZ_HEIGHT']),C.c_ulong(-1).value,2)
if not i:raise SystemExit('No image')
v=i.contents
print(v.width,v.height,v.bits_per_pixel)
if v.bits_per_pixel!=32:raise SystemExit('Unsupported pixel layout')
data=C.string_at(v.data,v.bytes_per_line*v.height)
print('nonzero bytes:',sum(b!=0 for b in data))
print(QImage(data,v.width,v.height,v.bytes_per_line,QImage.Format_RGB32).save('/tmp/rviz-composite.png'))
x.XDestroyImage.argtypes=[C.POINTER(XImage)];x.XDestroyImage(i)
x.XFreePixmap.argtypes=[C.c_void_p,C.c_ulong];x.XFreePixmap(d,p)
x.XCloseDisplay.argtypes=[C.c_void_p];x.XCloseDisplay(d)
