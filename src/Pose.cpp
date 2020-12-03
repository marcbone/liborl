// Copyright (c) 2020 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#include <liborl/Pose.h>
using namespace orl;
using namespace Eigen;



void Orientation::set_RPY(double r, double p, double y) {
    Eigen::Matrix3d a;
    a = AngleAxisd(y, Vector3d::UnitZ())
        * AngleAxisd(p, Vector3d::UnitY())
        * AngleAxisd(r, Vector3d::UnitX());
    quaternion = a;
    quaternion.normalize();

}

void Orientation::get_RPY(double &roll, double &pitch, double &yaw) const {
    //ZYX, yaw, pitch, roll
    Eigen::Vector3d rpy_vec = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);

    roll = rpy_vec.z();
    pitch = rpy_vec.y();
    yaw = rpy_vec.x();
}

std::string Orientation::toString() const {
    std::stringstream ss;
    double roll, pitch, yaw;
    get_RPY(roll, pitch, yaw);
    ss << "roll: " << roll << "\tpitch: " << pitch << "\tyaw: " << yaw;
    return ss.str();
}

Orientation::Orientation(const Quaterniond &quaternion) : quaternion(quaternion) {}

Orientation::Orientation() : quaternion() {}

Orientation::Orientation(double roll, double pitch, double yaw) : quaternion() {
    set_RPY(roll, pitch, yaw);
}

void Orientation::add_RPY(double r, double p, double y) {
    quaternion = Orientation(r, p, y).quaternion * quaternion;
}

Orientation Orientation::operator*(const Orientation &b) const {
    return Orientation(quaternion * b.quaternion);
}

Orientation Orientation::inverse() const {
    return Orientation(quaternion.inverse());
}

Pose::Pose() {
//    pose
//    orientation.set_RPY(0, 0, 0);
}

Pose::Pose(std::array<double, 16> mat) {
    set(mat);
}

std::ostream &Pose::operator<<(std::ostream &os) const {
    os << "position" << std::endl;
    return os;
}

std::array<double, 16> Pose::to_matrix() const {

    Matrix3d rot_mat = rotation();
    std::array<double, 16> mat = {rot_mat(0, 0), rot_mat(1, 0), rot_mat(2, 0), 0, rot_mat(0, 1), rot_mat(1, 1),
                                  rot_mat(2, 1), 0, rot_mat(0, 2), rot_mat(1, 2), rot_mat(2, 2), 0, translation().x(),
                                  translation().y(), translation().z(), 1};
    return mat;
}

Pose Pose::interpolate(const Pose &dest_pose, double progress) const {
    if (progress > 1) {
        progress = 1;
    }
    Eigen::Vector3d pos_result = translation() + (dest_pose.translation() - translation()) * progress;
    Eigen::Quaterniond quat_start, quat_end, quat_result;
    quat_start = rotation();
    quat_end = dest_pose.rotation();
    quat_result = quat_start.slerp(progress, quat_end);
    Pose result_pose(pos_result, quat_result);
    return result_pose;
}

void Pose::set(std::array<double, 16> mat) {
    Eigen::Vector3d position(mat[12], mat[13], mat[14]);
    Eigen::Affine3d::LinearMatrixType rot_mat;
    rot_mat << mat[0], mat[4], mat[8], mat[1], mat[5], mat[9], mat[2], mat[6], mat[10];
    pose = Eigen::Translation3d(position) * rot_mat;
}

std::string Pose::toString() const {
    std::stringstream ss;
    double roll, pitch, yaw;
    get_RPY(roll, pitch, yaw);

    ss << pose.matrix() << std::endl;
    ss << "roll: " << roll << "\tpitch: " << pitch << "\tyaw: " << yaw;
    return ss.str();
}

Pose::Pose(const Position &p, const Orientation &o) {
    pose = Eigen::Translation3d(p) * o.quaternion;
}

Pose::Pose(const Eigen::Vector3d &p, const Eigen::Quaterniond &o) {
    pose = Eigen::Translation3d(p) * o;
}

Position Pose::getPosition() const {
    return translation();
}

void Pose::setPosition(const Position &position) {
    pose = Eigen::Translation3d(position) * rotation();
}

Orientation Pose::getOrientation() const {
    return {Eigen::Quaterniond(rotation())};
}

void Pose::setOrientation(const Orientation &orientation) {
    pose = Eigen::Translation3d(translation()) * orientation.quaternion;
}

Pose::Pose(const Affine3d &pose) : pose(pose) {}

void Pose::set_RPY(double r, double p, double y) {
    Eigen::Affine3d a;
    a = AngleAxisd(y, Vector3d::UnitZ())
        * AngleAxisd(p, Vector3d::UnitY())
        * AngleAxisd(r, Vector3d::UnitX());
    pose = Eigen::Translation3d(translation()) * a;

}

void Pose::get_RPY(double &roll, double &pitch, double &yaw) const {
    //ZYX, yaw, pitch, roll
    Eigen::Vector3d rpy_vec = rotation().eulerAngles(2, 1, 0);

    roll = rpy_vec.z();
    pitch = rpy_vec.y();
    yaw = rpy_vec.x();
}

void Pose::set_position(double x, double y, double z) {
    setPosition(Position(x, y, z));
}

void Pose::add_position(double x, double y, double z) {
    pose = Eigen::Translation3d(x, y, z) * pose;
}


Pose Pose::operator*(const Pose &pose_b) const {
    return {pose * pose_b.pose};
}

Eigen::Quaterniond Pose::quaternion() const {
    Eigen::Quaterniond quat;
    quat = rotation();
    return quat;
}

Eigen::Vector3d Pose::translation() const {
    return pose.translation();
}

Eigen::Affine3d::LinearMatrixType Pose::rotation() const {
    return pose.rotation();
}

void Pose::add_RPY(double r, double p, double y) {
    setOrientation(Orientation(r, p, y) * quaternion());
}