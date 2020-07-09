#!/usr/bin/python3
import sys
import os
import shutil
import argparse

if __name__ == "__main__":

    # initialize argparse options
    parser = argparse.ArgumentParser()
    parser.add_argument('--clean', action='store_true', help='Clean build folder')
    parser.add_argument('--android', action='store_true', help='Only compile for android')
    parser.add_argument('--all', action='store_true', help='Only compile for all platforms')
    parser.add_argument('--run', dest= 'native_exe', help='run native executable')
    parser.add_argument('--run-android', dest= 'android_exe', help='run android executable, device connection required')

    # handle auto complete
    if len(sys.argv) == 2 and sys.argv[1] == '--complete-config':
        print(' '.join( parser._optionals._option_string_actions.keys()))
        quit()

    # create folder variables
    script_folder = os.path.abspath(os.path.dirname(__file__))
    build_dir = os.path.join(script_folder, 'build')
    android_build_dir = os.path.join(build_dir,'android')
    native_build_dir = os.path.join(build_dir,'native')
    android_work_dir = '/data/local/tmp/home/chenxx/'
    android_lib_dir = '/data/local/tmp/usr/lib/'

    # handle arguments
    args = parser.parse_args()
    if args.clean:
        shutil.rmtree(build_dir, ignore_errors=True)
        quit()

    target_platforms = {'native': False, 'android': False}
    if args.all:
        for k in target_platforms:
            target_platforms[k] = True
    elif args.android or args.android_exe is not None:
        target_platforms['android'] = True
    elif args.native_exe is not None:
        target_platforms['native'] = True

    # build native
    if target_platforms['native']:
        os.makedirs(native_build_dir, exist_ok=True)
        os.chdir(native_build_dir)
        os.system('cmake {}'.format(script_folder))
        os.system('make -j4')

    # build android
    if target_platforms['android']:
        os.makedirs(android_build_dir, exist_ok=True)
        os.chdir(android_build_dir)
        tool_chain_file = os.path.join(os.environ['ANDROID_NDK'],'build','cmake','android.toolchain.cmake')
        os.system('cmake {} -DCMAKE_TOOLCHAIN_FILE={} -DCMAKE_GENERATOR="Unix Makefiles" -DANDROID_PLATFORM=android-26'.format(script_folder, tool_chain_file))
        os.system('make -j4')

    # run native executable
    for _ in [1]:
        if args.native_exe is None:
            break
        local_exe_path = os.path.join(native_build_dir, args.native_exe)
        if not os.path.isfile(local_exe_path):
            print('Error: native executable {} not found!'.format(local_exe_path))
            break
        print("============ run native ============")
        os.system(local_exe_path)

    # run android executable
    for _ in [1]:
        if args.android_exe is None:
            break
        local_exe_path = os.path.join(android_build_dir, args.android_exe)
        if not os.path.isfile(local_exe_path):
            print('Error: android executable {} not found!'.format(local_exe_path))
            break

        device_exe_path = os.path.join(android_work_dir, os.path.basename(args.android_exe))
        os.system('adb shell "mkdir -p {}"'.format(android_work_dir))
        os.system('adb push {} {}'.format(local_exe_path, device_exe_path))
        os.system('adb shell chmod +x {}'.format(device_exe_path))
        print("============ run android ============")
        os.system('adb shell LD_LIBRARY_PATH={} {}'.format(android_lib_dir, device_exe_path))

