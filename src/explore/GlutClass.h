#ifndef __GLUTCLASS_H__
#define __GLUTCLASS_H__

#include "Robot.h"
#include "Utils.h"
#include <ctime>

class GlutClass
{
    public:
        static GlutClass* getInstance();

        void initialize();
        void process();
        void terminate();

        void screenshot(int doornumber, float w_alpha, int testNumber);

        void setRobot(Robot* r);

        bool drawRobotPath;

        int glutWindowSize;
        int frame;
        bool limitFrames;

        int halfWindowSize;
        int x_aux, y_aux;

    private:
        GlutClass ();
        static GlutClass* instance;

        Robot* robot_;
        Grid* grid_;
        Timer timer;

        int halfWindowSizeX_, halfWindowSizeY_;
        bool lockCameraOnRobot;

        int id_;

	    void render();

        static void display();
        static void reshape(int w, int h);
        static void keyboard(unsigned char key, int x, int y);
        static void specialKeys(int key, int x, int y);
};

#endif /* __GLUT_H__ */


