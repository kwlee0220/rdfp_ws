import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import argparse
import json
import glob


def get_default_config():
    """Get the default configuration for the camera calibration"""
    config = {
        'cam_file'          : '',
        'cam_fisheye'       : False,
        'cam_K'             : None,
        'cam_distort'       : None,
        'chess_pattern_cols': 9,
        'chess_pattern_rows': 6,
        'chess_cellsize'    : 0.03, # [m]
        'subpx_winsize'     : 11,
        'subpx_max_iter'    : 100,
        'subpx_eps'         : 0.001,
        'calib_option'      : [],
        'calib_output'      : '',
        'save_cam_pose'     : False,
        'img_plot_rows'     : 3,
    }
    return config

def get_calib_flags(calib_option, cam_fisheye=False):
    """Get the calibration flags from the given options"""
    calib_opts = [opt.upper() for opt in calib_option]
    calib_flags = 0
    if cam_fisheye:
        if 'CALIB_USE_INTRINSIC_GUESS'  in calib_opts: calib_flags += cv.fisheye.CALIB_USE_INTRINSIC_GUESS
        if 'CALIB_RECOMPUTE_EXTRINSIC'  in calib_opts: calib_flags += cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        if 'CALIB_CHECK_COND'           in calib_opts: calib_flags += cv.fisheye.CALIB_CHECK_COND
        if 'CALIB_FIX_SKEW'             in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_SKEW
        if 'CALIB_FIX_K1'               in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_K1
        if 'CALIB_FIX_K2'               in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_K2
        if 'CALIB_FIX_K3'               in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_K3
        if 'CALIB_FIX_K4'               in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_K4
        if 'CALIB_FIX_INTRINSIC'        in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_INTRINSIC
        if 'CALIB_FIX_PRINCIPAL_POINT'  in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_PRINCIPAL_POINT
        if 'CALIB_FIX_FOCAL_LENGTH'     in calib_opts: calib_flags += cv.fisheye.CALIB_FIX_FOCAL_LENGTH
    else:
        if 'CALIB_USE_INTRINSIC_GUESS'  in calib_opts: calib_flags += cv.CALIB_USE_INTRINSIC_GUESS
        if 'CALIB_FIX_PRINCIPAL_POINT'  in calib_opts: calib_flags += cv.CALIB_FIX_PRINCIPAL_POINT
        if 'CALIB_FIX_ASPECT_RATIO'     in calib_opts: calib_flags += cv.CALIB_FIX_ASPECT_RATIO
        if 'CALIB_ZERO_TANGENT_DIST'    in calib_opts: calib_flags += cv.CALIB_ZERO_TANGENT_DIST
        if 'CALIB_FIX_FOCAL_LENGTH'     in calib_opts: calib_flags += cv.CALIB_FIX_FOCAL_LENGTH
        if 'CALIB_FIX_K1'               in calib_opts: calib_flags += cv.CALIB_FIX_K1
        if 'CALIB_FIX_K2'               in calib_opts: calib_flags += cv.CALIB_FIX_K2
        if 'CALIB_FIX_K3'               in calib_opts: calib_flags += cv.CALIB_FIX_K3
        if 'CALIB_FIX_K4'               in calib_opts: calib_flags += cv.CALIB_FIX_K4
        if 'CALIB_FIX_K5'               in calib_opts: calib_flags += cv.CALIB_FIX_K5
        if 'CALIB_FIX_K6'               in calib_opts: calib_flags += cv.CALIB_FIX_K6
        if 'CALIB_RATIONAL_MODEL'       in calib_opts: calib_flags += cv.CALIB_RATIONAL_MODEL
        if 'CALIB_THIN_PRISM_MODEL'     in calib_opts: calib_flags += cv.CALIB_THIN_PRISM_MODEL
        if 'CALIB_FIX_S1_S2_S3_S4'      in calib_opts: calib_flags += cv.CALIB_FIX_S1_S2_S3_S4
        if 'CALIB_TILTED_MODEL'         in calib_opts: calib_flags += cv.CALIB_TILTED_MODEL
        if 'CALIB_FIX_TAUX_TAUY'        in calib_opts: calib_flags += cv.CALIB_FIX_TAUX_TAUY
    return calib_flags


def load_calib_config(config_file):
    """Load the configuration from the given file"""
    config = get_default_config()
    with open(config_file, 'rt') as f:
        config_new = json.load(f)
        config.update(config_new)

    if type(config['cam_K']) is list:
        config['cam_K'] = np.array(config['cam_K'])
    if type(config['cam_distort']) is list:
        config['cam_distort'] = np.array(config['cam_distort'])
    if (type(config['subpx_winsize']) is not tuple) or (type(config['subpx_winsize']) is not list):
        config['subpx_winsize'] = (config['subpx_winsize'], config['subpx_winsize'])
    config['subpx_criteria'] = [cv.TERM_CRITERIA_MAX_ITER + cv.TERM_CRITERIA_EPS, config['subpx_max_iter'], config['subpx_eps']]
    config['calib_flags'] = get_calib_flags(config['calib_option'], config['cam_fisheye'])
    return config


def load_calib_result(json_file):
    '''Load the calibration result from the json file.'''
    with open(json_file, 'r') as f:
        calib = json.load(f)
        calib['cam_K'] = np.array(calib['cam_K'])
        calib['cam_distort'] = np.array(calib['cam_distort'])
    return calib


def get_2d_points(given_files, chess_pattern_rows, chess_pattern_cols, subpx_winsize, subpx_criteria):
    """Get the 2D points from the given images"""
    cam_pts, cam_img, cam_files = [], [], []
    for file in given_files:
        # Load an image
        img = cv.imread(file)
        if len(img.shape) >= 3 and img.shape[2] == 3:
            img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

        # Extract corner points from the images
        ret, pts = cv.findChessboardCorners(img, (chess_pattern_cols, chess_pattern_rows))
        if ret:
            pts = cv.cornerSubPix(img, pts, subpx_winsize, (-1, -1), subpx_criteria)
            cam_pts.append(pts)
            cam_img.append(img)
            cam_files.append(file)
    return cam_pts, cam_img, cam_files


def get_3d_points(chess_pattern_rows, chess_pattern_cols, chess_cellsize):
    """Get the 3D points of the chessboard"""
    x, y = np.meshgrid(range(chess_pattern_cols), range(chess_pattern_rows))
    z = np.zeros_like(x)
    pts = chess_cellsize * np.dstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))
    return pts


def show_reproject_pts(cam_img, cam_fisheye, cam_K, cam_distort, cam_rvec, cam_tvec, cam_pts, obj_pts, n_rows):
    """Show the reprojected points on the images"""
    if n_rows > 0:
        if cam_fisheye:
            projectPoints = cv.fisheye.projectPoints
        else:
            projectPoints = cv.projectPoints
        plt.figure()
        for i in range(len(cam_img)):
            ax = plt.subplot(n_rows, max(int(len(cam_img) / n_rows + 0.5), 1), i + 1)
            ax.imshow(cam_img[i], cmap='gray')
            ax.plot(cam_pts[i][:,:,0], cam_pts[i][:,:,1], 'r+', label='extract', markersize=10)
            proj, _ = projectPoints(obj_pts[i], cam_rvec[i], cam_tvec[i], cam_K, cam_distort)
            ax.plot(proj[:,:,0], proj[:,:,1], 'b+', label='project', markersize=10)
            ax.axis('off')
        plt.tight_layout(pad=0, h_pad=0, w_pad=0)


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='camera calibration using chessboard images')
    parser.add_argument('config_file', type=str, help='path to JSON file containing the configuration')
    args = parser.parse_args()

    # Load configuration
    config = load_calib_config(args.config_file)

    # Extract 2D points on the given images
    if not config['cam_file']:
        raise('An image sequence for the camera is not given')
    all_files = sorted(glob.glob(config['cam_file']))
    cam_pts, cam_img, cam_files = get_2d_points(all_files, config['chess_pattern_rows'], config['chess_pattern_cols'], config['subpx_winsize'], config['subpx_criteria'])

    # Prepare 3D points
    chessboard = get_3d_points(config['chess_pattern_rows'], config['chess_pattern_cols'], config['chess_cellsize'])
    obj_pts = [chessboard.astype(np.float32)] * len(cam_img)

    # Calibrate the camera
    if config['cam_fisheye']:
        cam_rms, cam_K, cam_distort, cam_rvec, cam_tvec = cv.fisheye.calibrate(obj_pts, cam_pts, cam_img[0].shape[::-1], config['cam_K'], config['cam_distort'], flags=config['calib_flags'])
    else:
        cam_rms, cam_K, cam_distort, cam_rvec, cam_tvec = cv.calibrateCamera(obj_pts, cam_pts, cam_img[0].shape[::-1], config['cam_K'], config['cam_distort'], flags=config['calib_flags'])

    # Write the calibration result
    if config['calib_output']:
        with open(config['calib_output'], 'wt') as f:
            calib_result = {
                'cam_fisheye'   : config['cam_fisheye'],
                'cam_K'         : cam_K.tolist(),
                'cam_distort'   : cam_distort.flatten().tolist(),
                'rms'           : cam_rms,
            }
            if config['save_cam_pose']:
                calib_result['cam_pose'] = []
                for idx, file in enumerate(cam_files):
                    calib_result['cam_pose'].append({'file': file, 'rvec': cam_rvec[idx].flatten().tolist(), 'tvec': cam_tvec[idx].flatten().tolist()})
            json.dump(calib_result, f, indent=4)

    # Print the calibration result briefly
    print('### Brief Calibration Report')
    print(f'* Camera type: {"Kannala-Brandt (a.k.a. Fisheye)" if config["cam_fisheye"] else "Brown-Conrady"}')
    print(f'* Calibration flags: {bin(config["calib_flags"])}')
    print(f'* Camera files: {config["cam_file"]}')
    print(f'* The number of used images: {len(cam_files)} / {len(all_files)}')
    print(f'* RMS error: {cam_rms:.6f} [pixel]')

    # Visualize reprojected points
    show_reproject_pts(cam_img, config['cam_fisheye'], cam_K, cam_distort, cam_rvec, cam_tvec, cam_pts, obj_pts, config['img_plot_rows'])
    plt.show()
