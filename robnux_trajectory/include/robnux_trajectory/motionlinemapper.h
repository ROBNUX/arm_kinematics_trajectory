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
  // line of the motion command currently executing, -1 if none
  int get_cur_motionline();
  // line of the previously executed motion command, -1 if none
  int get_last_motionline();
  // called by the executor when it starts the next trajectory
  void advance();
  int addmotionline(const motionline_element_t &ml);
  void reset();
  std::queue<motionline_element_t> ml_;
  std::mutex mtx_;

 private:
  motionline_element_t cur_{-1, 0, 0, 0.0}, last_{-1, 0, 0, 0.0};
  int obj_id = 0;
};

#endif /* MOTIONLINEMAPPER_H_ */
