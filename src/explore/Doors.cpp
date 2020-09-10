#include "Doors.h"

#include <GL/glut.h>
#include <GL/freeglut.h>
#include <utility>  // std::pair, std::make_pair

Door::Door(std::string n)
{
    name=n;
    number=std::stoi(n);
    numObservations = 0;
}

bool Door::addNeighbor(Door *n)
{
    for(unsigned int i=0; i<neighbors.size(); i++){
        if(n==neighbors[i])
            return false;
    }
    neighbors.push_back(n);
    return true;
}

Doors::Doors()
{
    minValidObservations=3;
    newObservation=false;
    sizeOfRecentDoors = 8;
}

std::set<Door *> Doors::updateRight(const Pose &robotPose, const std::set<std::string> &observedDoorsRight)
{
    Door *d;
    std::set<std::string>:: iterator i;
    std::map<std::string,Door*>::iterator itDoor;

    std::set<Door*> doors;

//    std::cout << "RIGHT DOORS:";
    for(i = observedDoorsRight.begin(); i != observedDoorsRight.end(); i++){
        const std::string& doorName = *i;
//        std::cout<<" "<<*i;
        itDoor = doorsMap.find(doorName);
        if (itDoor == doorsMap.end()){
            // the door is not in the map
            // creates new door, since it is the first time that the robot observes it
            d = new Door(doorName);
            doorsMap.insert(std::pair<std::string, Door*>(doorName,d));
        }else{
            d = itDoor->second;
        }

        double angle = robotPose.theta-90.0;
        double dx = cos(DEG2RAD(angle))*3.0;
        double dy = sin(DEG2RAD(angle))*3.0;

        d->sumPoses.x += robotPose.x + dx;
        d->sumPoses.y += robotPose.y + dy;

        d->sum_map_poses.x += robotPose.x;
        d->sum_map_poses.y += robotPose.y;

        d->numObservations++;

        d->pose.x = d->sumPoses.x/(double)d->numObservations;
        d->pose.y = d->sumPoses.y/(double)d->numObservations;

        d->map_pose.x = d->sum_map_poses.x/(double)d->numObservations;
        d->map_pose.y = d->sum_map_poses.y/(double)d->numObservations;

        doors.insert(d);

//        std::cout << d->name << " (" << d->numObservations << ") ";
    }

    return doors;

}

std::set<Door*> Doors::updateLeft(const Pose &robotPose, const std::set<std::string> &observedDoorsLeft)
{
    Door *d;
    std::set<std::string>:: iterator i;
    std::map<std::string,Door*>::iterator itDoor;

    std::set<Door*> doors;

//    std::cout << "LEFT DOORS:";
    for(i = observedDoorsLeft.begin(); i != observedDoorsLeft.end(); i++){
        const std::string& doorName = *i;
//        std::cout<<" "<<*i;
        itDoor = doorsMap.find(doorName);
        if (itDoor == doorsMap.end()){
            // the door is not in the map
            // creates new door, since it is the first time that the robot observes it
            d = new Door(doorName);
            doorsMap.insert(std::pair<std::string, Door*>(doorName,d));
        }else{
            d = itDoor->second;
        }

        double angle = robotPose.theta+90.0;
        double dx = cos(DEG2RAD(angle))*3.0;
        double dy = sin(DEG2RAD(angle))*3.0;
//        std::cout << "(" << angle << ") " << dx << " " << dy
//                  << " [" << RAD2DEG(atan2(dy,dx)) << "]" << std::endl;

        d->sumPoses.x += robotPose.x + dx;
        d->sumPoses.y += robotPose.y + dy;

        d->sum_map_poses.x += robotPose.x;
        d->sum_map_poses.y += robotPose.y;

        d->numObservations++;

        d->pose.x = d->sumPoses.x/(double)d->numObservations;
        d->pose.y = d->sumPoses.y/(double)d->numObservations;

        d->map_pose.x = d->sum_map_poses.x/(double)d->numObservations;
        d->map_pose.y = d->sum_map_poses.y/(double)d->numObservations;
        doors.insert(d);

//        std::cout << d->name << " (" << d->numObservations << ") ";
    }

    return doors;

}

void Doors::print()
{
    Door *d;
    std::map<std::string,Door*>::iterator itDoor;

    std::cout << "Map Size " << doorsMap.size() << '\n';
    for (itDoor=doorsMap.begin(); itDoor!=doorsMap.end(); ++itDoor){
        d = itDoor->second;
        std::cout << d->name << " => " << d->numObservations << '\n';
    }
}

void Doors::updateRecent(const std::set<std::string> &observedDoorsLeft, const std::set<std::string> &observedDoorsRight)
{
    std::set<std::string>:: iterator itObserved;
    std::list<std::pair<std::string,int> >::iterator itRecent;

    int numOdds=0;
    int numEvens=0;
    double sumDoorsNumbers=0;

    int count=0, max = observedDoorsLeft.size()+observedDoorsRight.size();

    for(itObserved = observedDoorsLeft.begin(); itObserved != observedDoorsLeft.end(); itObserved++){
        const std::string& doorName = *itObserved;

        int number = std::stoi(doorName);
        if(number%2==0)
            numEvens++;
        else
            numOdds++;
        sumDoorsNumbers += number;

        bool found = false;
//        std::cout<<"LEFT - RECENT DOORS:";
        for (itRecent=recentDoors.begin(); itRecent != recentDoors.end() && count<max; ++itRecent){
//            std::cout<<" "<<itRecent->first;
            if(doorName == (*itRecent).first){
                (*itRecent).second += 1;
                if((*itRecent).second==minValidObservations)
                    newObservation=true;
                found = true;
                break;
            }
            count++;
        }
//        std::cout<<std::endl;

        if(!found){
            recentDoors.push_front(std::pair<std::string,int>(doorName,1));
            if(recentDoors.size() > sizeOfRecentDoors){
                recentDoors.pop_back();
            }
        }
    }

    for(itObserved = observedDoorsRight.begin(); itObserved != observedDoorsRight.end(); itObserved++){
        const std::string& doorName = *itObserved;

        int number = std::stoi(doorName);
        if(number%2==0)
            numEvens++;
        else
            numOdds++;
        sumDoorsNumbers += number;

        bool found = false;
//        std::cout<<"RIGHT - RECENT DOORS:";
        for (itRecent=recentDoors.begin(); itRecent != recentDoors.end() && count<max; ++itRecent){
//            std::cout<<" "<<itRecent->first;
            if(doorName == (*itRecent).first){
                (*itRecent).second += 1;
                if((*itRecent).second==minValidObservations)
                    newObservation=true;
                found = true;
                break;
            }
            count++;
        }
//        std::cout<<std::endl;

        if(!found){
            recentDoors.push_front(std::pair<std::string,int>(doorName,1));
            if(recentDoors.size() > sizeOfRecentDoors){
                recentDoors.pop_back();
            }
        }
    }

    numEvenRoomsNearRobot = numEvens;
    numOddRoomsNearRobot = numOdds;
    avgDoorNumberNearRobot = sumDoorsNumbers/(numEvens+numOdds);

//    std::cout << "LAST OBSERVED ";
//    for (itRecent=recentDoors.begin(); itRecent != recentDoors.end(); ++itRecent){
//        std::cout << (*itRecent).first << "(" << (*itRecent).second << ") ";
//    }
//    std::cout << std::endl;
}

void Doors::draw(double scale)
{
    Door *d;
    std::map<std::string,Door*>::iterator itDoor;

    glScalef(scale,scale,scale);

    glColor3f(0.5f, 0.0f, 0.5f);
    for (itDoor=doorsMap.begin(); itDoor!=doorsMap.end(); ++itDoor){
        d = itDoor->second;

        drawNumbers(d->name,d->pose.x,d->pose.y);

        //drawNumbers(d->name,d->map_pose.x,d->map_pose.y);

//        glRasterPos2f(d->pose.x+0.25, d->pose.y+0.25);

//        for (unsigned int i=0; i<d->name.size(); i++)
//        {
//            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, d->name[i]);
//        }
    }

    glScalef(1.0/scale,1.0/scale,1.0/scale);

}

void DrawCircle(float cx, float cy, float r, int num_segments)
{
    float theta = 2 * 3.1415926 / float(num_segments);
    float c = cos(theta);
    float s = sin(theta);
    float t;

    float x = r;//we start at angle = 0
    float y = 0;

    glColor3f(1.0,1.0,0.0);
    glBegin(GL_POLYGON);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x + cx, y + cy);//output vertex

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();

    x = r;
    y = 0;

    glColor3f(0.0,0.0,0.0);
    glLineWidth(3);
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x + cx, y + cy);//output vertex

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();
}

void Doors::drawGraph(double scale)
{
    Door *d;
    std::map<std::string,Door*>::iterator itDoor;

    glScalef(scale,scale,scale);

    // Draw edges
    glColor3f(0.3,0.0,0.0);
//    glLineWidth(5);
//    glBegin(GL_LINES);
//    for (itDoor=doorsMap.begin(); itDoor!=doorsMap.end(); ++itDoor){
//        d = itDoor->second;

//        for(int n=0; n<d->neighbors.size(); n++){
//            glVertex2d(d->pose.x,d->pose.y);
//            glVertex2d(d->neighbors[n]->pose.x,d->neighbors[n]->pose.y);
//        }
//    }
//    glEnd();

    // Draw nodes
    for (itDoor=doorsMap.begin(); itDoor!=doorsMap.end(); ++itDoor){
        d = itDoor->second;

        DrawCircle(d->pose.x,d->pose.y,0.8*1.5,30);

        drawNumbers(d->name,d->pose.x,d->pose.y);

//        DrawCircle(d->map_pose.x,d->map_pose.y,0.8,30);

//        drawNumbers(d->name,d->map_pose.x,d->map_pose.y);
    }

    glScalef(1.0/scale,1.0/scale,1.0/scale);

}
