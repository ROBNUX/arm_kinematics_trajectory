#include "robnux_trajectory/motionlinemapper.h"

motionlinemapper::motionlinemapper() {
  // TODO Auto-generated constructor stub
}

motionlinemapper::~motionlinemapper() {
  // TODO Auto-generated destructor stub
}

motionlinemapper::motionlinemapper(const motionlinemapper &other) {
  // TODO Auto-generated constructor stub
}

int motionlinemapper::get_cur_motionline() {
  std::lock_guard<std::mutex> lock(mtx_);
  return cur_.line;
}

int motionlinemapper::get_last_motionline() {
  std::lock_guard<std::mutex> lock(mtx_);
  return last_.line;
}

void motionlinemapper::advance() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (ml_.empty()) return;
  last_ = cur_;
  cur_ = ml_.front();
  ml_.pop();
}

motionlinemapper::motionlinemapper(const int &objid) { obj_id = objid; }

int motionlinemapper::addmotionline(const motionline_element_t &ml) {
  mtx_.lock();
  ml_.push(ml);
  mtx_.unlock();
  return 0;
}

void motionlinemapper::set_control_obj(int id) { obj_id = id; }

void motionlinemapper::reset() {
  std::queue<motionline_element_t> ml_empty;
  mtx_.lock();
  std::swap(ml_empty, ml_);
  cur_ = last_ = motionline_element_t{-1, 0, 0, 0.0};
  mtx_.unlock();
}
