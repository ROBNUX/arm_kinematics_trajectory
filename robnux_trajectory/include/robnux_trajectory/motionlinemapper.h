/*
 * motionlinemapper.h
 *
 *  Created on: 2022年1月18日
 *      Author: chi
 */

#ifndef MOTIONLINEMAPPER_H_
#define MOTIONLINEMAPPER_H_

#include <stdint.h>
#include <stdlib.h>

#include <mutex>
#include <queue>
#include <vector>
using namespace std;
typedef struct motionline_element {
  int line;
  uint64_t start_index;
  uint64_t end_index;
  double duration;
} motionline_element_t;

class motionlinemapper {
 public:
  motionlinemapper();
  void set_control_obj(int id);
  motionlinemapper(const int &objid);
  virtual ~motionlinemapper();
  motionlinemapper(const motionlinemapper &other);
  int get_cur_motionline();
  int addmotionline(const motionline_element_t &ml);
  void reset();
  std::queue<motionline_element_t> ml_;
  std::mutex mtx_;

 private:
  motionline_element_t cur_, last_;
  int obj_id = 0;
};

#endif /* MOTIONLINEMAPPER_H_ */
