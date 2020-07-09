#!/usr/bin/python3
import sys
import os
import shutil
import argparse

if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == '--complete-config':
        print('--help -h --clean --android --all')
        quit()

    script_folder = os.path.abspath(os.path.dirname(__file__))
    build_dir = os.path.join(script_folder, 'build')
    android_build_dir = os.path.join(build_dir,'android')
    native_build_dir = os.path.join(build_dir,'native')
    android_work_dir = '/data/local/tmp/home/chenxx/'

    parser = argparse.ArgumentParser()
    parser.add_argument('--clean', action='store_true', help='Clean build folder')
    parser.add_argument('--android', action='store_true', help='Only compile for android')
    parser.add_argument('--all', action='store_true', help='Only compile for all platforms')
    args = parser.parse_args()

    if args.clean:
        shutil.rmtree(build_dir, ignore_errors=True)
        quit()

    target_platforms = {'native': True, 'android': False}
    if args.all:
        for k in target_platforms:
            target_platforms[k] = True
    elif args.android:
        for k in target_platforms:
            target_platforms[k] = False
        target_platforms['android'] = True

    if target_platforms['native']:
        os.makedirs(native_build_dir, exist_ok=True)
        os.chdir(native_build_dir)
        os.system('cmake {}'.format(script_folder))
        os.system('make -j4')

    if target_platforms['android']:
        os.makedirs(android_build_dir, exist_ok=True)
        os.chdir(android_build_dir)
        tool_chain_file = os.path.join(os.environ['ANDROID_NDK'],'build','cmake','android.toolchain.cmake')
        os.system('cmake {} -DCMAKE_TOOLCHAIN_FILE={} -DCMAKE_GENERATOR="Unix Makefiles" -DANDROID_PLATFORM=android-26'.format(script_folder, tool_chain_file))
        os.system('make -j4')

        exe_file = None #'hello.out'
        if exe_file is not None:
            local_exe_path = os.path.join(android_build_dir, exe_file)
            device_exe_path = os.path.join(android_work_dir, exe_file)
            os.system('adb shell "mkdir -p {}"'.format(android_work_dir))
            os.system('adb push {} {}'.format(local_exe_path, device_exe_path))
            os.system('adb shell chmod +x {}'.format(device_exe_path))
            os.system('adb shell LD_LIBRARY_PATH={} {}'.format(android_work_dir, device_exe_path))

