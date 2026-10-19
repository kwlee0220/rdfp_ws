from __future__ import annotations

from os import PathLike

import argparse
import json

import numpy as np
import cv2 as cv

__all__ = ['create_undistorter', 'ImageUndistorter', 'undistort_video']


def load_calib_file(json_file: str | PathLike[str]) -> dict:
    '''Load the calibration result from the json file.'''
    with open(json_file, 'r') as f:
        calib = json.load(f)
        calib['cam_K'] = np.array(calib['cam_K'])
        calib['cam_distort'] = np.array(calib['cam_distort'])
    return calib

def create_undistorter(calib_file: str | PathLike[str], **kwargs) -> 'ImageUndistorter':
    """Create an undistorter from the calibration result file (JSON)"""
    calib = load_calib_file(calib_file)
    return ImageUndistorter(calib['cam_K'], calib['cam_distort'], calib['cam_fisheye'], **kwargs)


class ImageUndistorter:
    def __init__(self, camera_K, camera_distort, is_fisheye=False, map_type=cv.CV_16SC2,
                 interpolation=cv.INTER_LINEAR):
        self.is_fisheye = is_fisheye
        self.cam_K = camera_K
        self.cam_distort = camera_distort
        self.map_type = map_type            # `cv.CV_16SC2` (fast, 1/32 pixel) or `cv.CV_32FC1` (precise)
        self.interpolation = interpolation
        self._map_size = None
        self._map1, self._map2 = None, None

    def init_maps(self, img_size):
        """Build the undistortion maps for the given image size, (width, height)"""
        img_size = tuple(int(v) for v in img_size)
        if self.is_fisheye:
            self._map1, self._map2 = cv.fisheye.initUndistortRectifyMap(self.cam_K, self.cam_distort, None, self.cam_K, img_size, self.map_type)
        else:
            self._map1, self._map2 = cv.initUndistortRectifyMap(self.cam_K, self.cam_distort, None, self.cam_K, img_size, self.map_type)
        self._map_size = img_size

    def undistort(self, img):
        """Undistort the given image using the maps, which are rebuilt only when the image size changes"""
        img_size = img.shape[1::-1]
        if img_size != self._map_size:
            self.init_maps(img_size)
        return cv.remap(img, self._map1, self._map2, self.interpolation)


def undistort_video(video_file, calib_file, output_video_file):
    """Undistort the video file and save it to the output video file. Return the number of written frames."""
    # Open the video file.
    video = cv.VideoCapture(video_file)
    assert video.isOpened(), f'Error: Cannot open video file, {video_file}.'
    fps = video.get(cv.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30
        print(f'Warning: Cannot get FPS of {video_file}, so {fps} FPS is used.')

    # Load the calibration data.
    undistorter = create_undistorter(calib_file)

    # Undistort the images and write them.
    # (The output video is opened with the size of the first frame, because `cv.VideoWriter` silently drops
    #  the frames whose size is different from the given size.)
    output = None
    n_frames = 0
    try:
        while True:
            success, img = video.read()
            if not success:
                break

            img = undistorter.undistort(img)
            if output is None:
                output = cv.VideoWriter(output_video_file, cv.VideoWriter_fourcc(*'mp4v'), fps, img.shape[1::-1])
                assert output.isOpened(), f'Error: Cannot open output video file, {output_video_file}.'
            output.write(img)
            n_frames += 1
    finally:
        video.release()
        if output is not None:
            output.release()
    return n_frames


def play_undistort(video_file, calib_file, wait_msec=1):
    """Play the video file with undistortion."""
    # Open the video file.
    video = cv.VideoCapture(video_file)
    assert video.isOpened(), f'Error: Cannot open video file, {video_file}.'

    # Load the calibration data, and build the undistortion maps once, which are reused for all frames.
    # (If the video does not report its size, they are built on the first frame.)
    undistorter = create_undistorter(calib_file)
    video_size = (video.get(cv.CAP_PROP_FRAME_WIDTH), video.get(cv.CAP_PROP_FRAME_HEIGHT))
    if min(video_size) > 0:
        undistorter.init_maps(video_size)

    # Undistort the images and show them.
    is_undistort = True
    while True:
        success, img = video.read()
        if not success:
            break

        if is_undistort:
            img = undistorter.undistort(img)

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