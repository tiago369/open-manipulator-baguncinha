/*
MIT License

Copyright (c) 2020 Etevaldo Cardoso

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <marker_localization/marker_pose_estimator.h>

bir::MarkerPoseEstimator::MarkerPoseEstimator(cv::Mat& camera_matrix, cv::Mat& distortion_coef)
  : expectedMarkers_(std::vector<std::pair<int, std::vector<int>>>())
  , cameraMatrix_(camera_matrix)
  , distCoeffs_(distortion_coef)
{
}

bir::MarkerPoseEstimator::MarkerPoseEstimator(const std::vector<std::pair<int, std::vector<int>>>& expected_markers,
                                              cv::Mat& camera_matrix, cv::Mat& distortion_coef)
  : expectedMarkers_(expected_markers), cameraMatrix_(camera_matrix), distCoeffs_(distortion_coef)
{
}

void bir::MarkerPoseEstimator::setExpectedMarkers(const std::vector<std::pair<int, std::vector<int>>>& exp_markers)
{
  expectedMarkers_ = exp_markers;
}

bir::MarkerTransformVector bir::MarkerPoseEstimator::estimatePose(const bir::MarkerVector& marker_vector)
{
  bir::MarkerTransformVector markers_transforms;

  if (marker_vector.isEmpty())
    return markers_transforms;

  std::vector<cv::Vec3d> rotationValues, translationValues;

  rotationValues.reserve(marker_vector.size());
  translationValues.reserve(marker_vector.size());
  markers_transforms.getAreas().reserve(marker_vector.size());
  markers_transforms.getIDs().reserve(marker_vector.size());
  markers_transforms.getCorners().reserve(marker_vector.size());

  const std::vector<cv::Point2f> emptyCorners = { cv::Point2f(), cv::Point2f(), cv::Point2f(), cv::Point2f() };

  getRotationAndTranslationValues(marker_vector, rotationValues, translationValues, markers_transforms.getAreas(),
                                  markers_transforms.getIDs(), markers_transforms.getCorners());

  const size_t markers_transforms_size = markers_transforms.size();

  markers_transforms.getTransforms().reserve(markers_transforms_size);
  markers_transforms.getProjectionsErros().reserve(markers_transforms_size);

  for (int index = 0; index < markers_transforms_size; index++)
  {
    tf2::Transform transform;

    const double translation[3] = { translationValues.at(index)[0], translationValues.at(index)[1],
                                    translationValues.at(index)[2] };

    transform.setOrigin(tf2::Vector3(translation[0], translation[1], translation[2]));
    transform.setRotation(quaternionFromRodrigues(rotationValues.at(index)));

    markers_transforms.getTransforms().push_back(std::move(transform));
    markers_transforms.getCorners().push_back(emptyCorners);
    markers_transforms.getProjectionsErros().push_back(0.00);  // todo create a method to get the projection error.
  }

  return markers_transforms;
}

void bir::MarkerPoseEstimator::getRotationAndTranslationValues(const bir::MarkerVector& marker_vector,
                                                               std::vector<cv::Vec3d>& p_rotation_values,
                                                               std::vector<cv::Vec3d>& p_translation_values,
                                                               std::vector<float>& area, std::vector<int>& ids,
                                                               std::vector<std::vector<cv::Point2f>> corners)
{
  if (marker_vector.isEmpty())
    return;

  for (auto marker_type : expectedMarkers_)
  {
    std::vector<std::vector<cv::Point2f>> corner;
    fillUpCorner(marker_vector, marker_type.second, corner, ids, area);

    std::vector<cv::Vec3d> rotationValues, translationValues;
    try
    {
      cv::aruco::estimatePoseSingleMarkers(corner,
                                           marker_type.first * 1E-3,  // Convert mm to m
                                           cameraMatrix_, distCoeffs_, rotationValues, translationValues);
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("getRotationAndTranslationValues: %s", e.what());
    }

    corners.insert(corners.end(), corner.begin(), corner.end());
    p_rotation_values.insert(p_rotation_values.end(), rotationValues.begin(), rotationValues.end());
    p_translation_values.insert(p_translation_values.end(), translationValues.begin(), translationValues.end());
  }
}

void bir::MarkerPoseEstimator::fillUpCorner(const bir::MarkerVector& marker_vector, const std::vector<int>& markers_id,
                                            std::vector<std::vector<cv::Point2f>>& corner_container,
                                            std::vector<int>& ids_orders, std::vector<float>& area)
{
  for (int id : markers_id)
  {
    if (marker_vector.hasID(id))
    {
      try
      {
        bir::Marker marker = marker_vector.withID(id);
        ids_orders.push_back(marker.id);
        corner_container.push_back(std::move(marker.corner));
        area.push_back(cv::contourArea(corner_container.back()));
      }
      catch (std::exception& e)
      {
        ROS_ERROR("fillUpCorner: %s", e.what());
      }
    }
  }
}
