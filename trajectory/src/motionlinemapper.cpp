#include "trajectory/motionlinemapper.h"

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
  //	if(ml_.size()<=0){
  //		return 0;
  //	}
  //	int lline=0;
  //	int  lcurid = amd_gr_get_cur_cmdid(obj_id);
  //	mtx_.lock();
  //	int lmlsize = ml_.size();
  //	for(int i=0;i<lmlsize;i++){
  //		cur_ = *ml_.front();
  //		if(cur_.start_index <= lcurid && cur_.end_index >= lcurid){
  //			//bingo
  //			lline= cur_.line;
  //		}else if(cur_.end_index < lcurid){
  //			ml_.pop();
  //		}
  //	}
  //	mtx_.unlock();

  //	return lline;
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
  mtx_.unlock();
}
