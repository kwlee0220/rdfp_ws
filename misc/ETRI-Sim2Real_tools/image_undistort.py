import argparse
import json
import numpy as np
import cv2 as cv
from camera_calib_chess import load_calib_result


def play_undistort(video_file, calib_file, wait_msec=1):
    """Play the video file with undistortion."""
    # Open the video file.
    video = cv.VideoCapture(video_file)
    assert video.isOpened(), f'Error: Cannot open video file, {video_file}.'

    # Load the calibration data.
    calib = load_calib_result(calib_file)

    # Undistort the images and show them.
    is_undistort = True
    while True:
        success, img = video.read()
        if not success:
            break

        if is_undistort:
            if calib['cam_fisheye']:
                img = cv.fisheye.undistortImage(img, calib['cam_K'], calib['cam_distort'], None, calib['cam_K'], img.shape[1::-1])
            else:
                img = cv.undistort(img, calib['cam_K'], calib['cam_distort'])

        cv.imshow('image_undistort', img)
        key = cv.waitKey(wait_msec)
        if key == ord('\t'):
            is_undistort = not is_undistort
        elif key == ord(' '):
            key = cv.waitKey(0)
        if key == 27:
            break
    if key != 27:
        cv.waitKey(0)


if __name__ == '__main__':
    # Test cases
    # play_undistort('data/TwoCameras/ISAWEdge_object.mp4', 'data/TwoCameras_sample/ISAWEdge_chess_calib_k0.json')
    # play_undistort('data/TwoCameras/ISAWEdge_object.mp4', 'data/TwoCameras_sample/ISAWEdge_chess_calib_k1.json')
    # exit()

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='image undistortion using camera calibration data')
    parser.add_argument('video_file', type=str, help='path to video file')
    parser.add_argument('calib_file', type=str, help='path to camera calibration file')
    parser.add_argument('--wait_msec', '-w', default=1, type=int, help='waiting time (after `cv.imshow`) in milliseconds')
    args = parser.parse_args()
    play_undistort(args.video_file, args.calib_file, args.wait_msec)