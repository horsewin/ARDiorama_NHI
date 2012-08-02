#ifndef CONTROLS_H
#define CONTROLS_H

class Controller {
public:
	Controller(bt_ARMM_world *m_world) { world = m_world; }
	void check_input();
protected:
	bt_ARMM_world *world;

};


#endif