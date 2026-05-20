/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-09-05 23:22:31
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2021-10-15 14:58:34
 */
#ifndef     __NAV_UTILS_H__
#define     __NAV_UTILS_H__

#include <iostream>
#include <math.h>
#include <vector>
#include <xutils.h>
#ifdef _WIN32
#else
#include <sys/time.h>
#include <unistd.h>
#endif




enum class CliffState {

    None = 0,
    LL = 1, 
    L = 2,
    R = 3, 
    RR = 4,
};

enum class BumpState {
    None = 0,
    LeftBumped = 1,
    RightBumped = 2,
    BothBumped = 3,
};

enum class OrientationState {
    Left = 0,
    Right = 1,
    Middle = 2
};

enum class ObsByPassState
{
    CommonAction,
    AlongWall,
    ObsByPass,
    BackToTrackPath,
    MiniLocal,
    Exit,
    TimeOut,
    None,
    AlongWallDone,
    ObsByPassDone,
    BackToTrackPathDone
};

enum class CleaningState
{
    Normal,
    TrackLine,
    ObsByPassRtn,
    CleanPatch,
    Mute
};

typedef struct NavPose
{
    double px;
    double py;
    double pa;

    NavPose():px(0),py(0),pa(0){}
    NavPose(double x_p,double y_p,double a_p):px(x_p),py(y_p),pa(a_p){}
    NavPose(const NavPose& p){
        px=p.px;
        py=p.py;
        pa=p.pa;
    }
    const NavPose operator+(const NavPose& p) {
        NavPose tmp;
        tmp.px = px + p.px;
        tmp.py = py + p.py;
        tmp.pa = pa + p.pa;
        return tmp;
    }
    const NavPose operator+(const NavPose& p) const {
        NavPose tmp;
        tmp.px = px + p.px;
        tmp.py = py + p.py;
        tmp.pa = pa + p.pa;
        return tmp;
    }

    const NavPose operator-(const NavPose& p) {
        NavPose tmp;
        tmp.px = px - p.px;
        tmp.py = py - p.py;
        tmp.pa = pa - p.pa;
        return tmp;
    }

    const NavPose operator-(const NavPose& p) const {
        NavPose tmp;
        tmp.px = px - p.px;
        tmp.py = py - p.py;
        tmp.pa = pa - p.pa;
        return tmp;
    }

    NavPose& operator=(const NavPose& p) {
        px = p.px;
        py = p.py;
        pa = p.pa;
        return *this;
    }
    bool operator== (const NavPose &p2)
    {
        return ((fabs(px-p2.px)<1e-3) && (fabs(py - p2.py)<1e-3));
    }
}NavPose;

typedef struct NavPoint
{
    double px;
    double py;
    NavPoint():px(0), py(0){}
    NavPoint(double x_p, double y_p) :px(x_p), py(y_p) {}
    NavPoint(const NavPoint& p)
    {
        px=p.px;
        py=p.py;
    }
    const NavPoint operator+(const NavPoint& p) {
        NavPoint tmp(0,0);
        tmp.px=px + p.px;
        tmp.py=py + p.py;
        return tmp;
    }
    

    const NavPoint operator-(const NavPoint& p) {
       NavPoint tmp(0,0);
       tmp.px=px - p.px;
       tmp.py=py - p.py;
       return tmp;
    }

    NavPoint& operator=(const NavPoint& p) {
        px = p.px;
        py = p.py;
        return *this;
    }
    const NavPoint operator/(double factor) {
        NavPoint tmp;
        if (fabs(factor) < 1e-7) {
            std::cerr << "Canot devide 0" << std::endl;
            return *this;
        }
        tmp.px = px / factor;
        tmp.py = py / factor;
        return tmp;
    }
    const NavPoint operator*(double factor) {
        NavPoint tmp;
        tmp.px = px * factor;
        tmp.py = py * factor;
        return tmp;
    }
    bool operator== (const NavPoint &p2)
    {
        return ((fabs(px-p2.px)<1e-3) && (fabs(py - p2.py)<1e-3));
    }

    double norm() {
        return sqrt(pow(px, 2) + pow(py, 2));
    }

    NavPoint normalized()
    {
        if(norm()<1e-7)
        {
            return NavPoint(0,0);
        }
        return NavPoint(px/norm(),py/norm());
    }
    double DotProduct(const NavPoint& p)
    {
        return px*p.px+py*p.py;
    }
    double DotProduct(const NavPoint& start,const NavPoint& end)
    {
        return px*(end.px-start.px)+py*(end.py-start.py);
    }
    NavPoint& operator+=(const NavPoint& p) {
        px += p.px;
        py += p.py;
        return *this;
    }
    NavPoint& operator-=(const NavPoint& p) {
        px -= p.px;
        py -= p.py;
        return *this;
    }

    double dotMul(NavPoint p1, NavPoint p2)
    {
        return p1.px*p2.px + p1.py * p2.py;
    }
    double crossMul(NavPoint p1, NavPoint p2)
    {
        return p1.px*p2.py - p2.px*p1.py;
    }

}NavPoint;

typedef struct LaserData
{
    float range;
    float bearing;
    int index;
    LaserData():range(0), bearing(0),index(-1){}
    LaserData(float range_p, float bearing_p,int index_p) :range(range_p), bearing(bearing_p),index(index_p) {}
    LaserData(const LaserData& p)
    {
        range = p.range;
        bearing = p.bearing;
        index=p.index;
    }
    LaserData& operator=(const LaserData& p) {
        range = p.range;
        bearing = p.bearing;
        index=p.index;
        return *this;
    }
}LaserData;

typedef struct LineData
{
    float distance;
    float dis2mid;
    float length;
    float local_angle;
    NavPoint start_point;
    NavPoint end_point;
    NavPoint proj_point;
    LineData():distance(0),dis2mid(0),length(0),local_angle(0){};
    LineData(float dis_p,float dis_mid,float lens_p,float angle_p, NavPoint& start,NavPoint& end,NavPoint& proj):
        distance(dis_p),
        dis2mid(dis_mid),
        length(lens_p),
        local_angle(angle_p),
        start_point(start),
        end_point(end),
        proj_point(proj)
        {};
    LineData(const LineData& l)
    {
        distance=l.distance;
        dis2mid=l.dis2mid;
        length=l.length;
        local_angle=l.local_angle;
        start_point=l.start_point;
        end_point=l.end_point;
        proj_point=l.proj_point;
    }
    LineData& operator=(const LineData& l) {
        distance=l.distance;
        dis2mid=l.dis2mid;
        length=l.length;
        local_angle=l.local_angle;
        start_point=l.start_point;
        end_point=l.end_point;
        proj_point=l.proj_point;
        return *this;
    }
}LineData;

typedef struct GridPoint
{
    int32_t x;
    int32_t y;
    GridPoint():x(0),y(0){}
    GridPoint(int32_t x_p,int32_t y_p):x(x_p),y(y_p){}
    GridPoint(const GridPoint& p)
    {
        x=p.x;
        y=p.y;
    }
    GridPoint& operator=(const GridPoint& p) {
        x = p.x;
        y = p.y;
        return *this;
    }
    const GridPoint operator+(const GridPoint& p) {
        GridPoint tmp;
        tmp.x = x + p.x;
        tmp.y = y + p.y;
        return tmp;
    }
    const GridPoint operator+(const GridPoint& p) const {
        GridPoint tmp;
        tmp.x = x + p.x;
        tmp.y = y + p.y;
        return tmp;
    }

    const GridPoint operator-(const GridPoint& p) {
        GridPoint tmp;
        tmp.x = x - p.x;
        tmp.y = y - p.y;
        return tmp;
    }
    bool operator==(const GridPoint& p) {
        if(x==p.x && y==p.y)
        {
            return true;
        }
        return false;
    }
    GridPoint& operator+=(const GridPoint& p) {
        x += p.x;
        y += p.y;
        return *this;
    }
    GridPoint& operator-=(const GridPoint& p) {
        x -= p.x;
        y -= p.y;
        return *this;
    }
}GridPoint;

typedef struct GridPose
{
    int32_t x;
    int32_t y;
    double a;
    GridPose():x(0),y(0),a(0){}
    GridPose(int32_t x_p,int32_t y_p,double a_p):x(x_p),y(y_p),a(a_p){}
}GridPose;

enum class NavPriority
{
    Low = 0,    // Idle
    Normal,     // Clean, Dock, SpotClean
    Medium,     // HandleSelfTrapError
    High,       // HandleError
    ExtraHigh,  // Error, Shutdown
    UltraHigh
};

enum class NavState
{
    Idle = 0,
    Clean,
    RemoteClean,
    GoToTarget,
    ZonedClean,
    SegmentClean,
    BackToDock,
    ReconnectToDock,
    SpotClean,
    MobilityTest,
    ResumeMobilityTest,
    DPUTest,
    HandleBumperError,
    HandleDropError,
    HandleTrapError,
    HandleOCError,
    HandleMotorError,
    HandleSensorError,
    HandleSelfTrapError,
    HandleGyroError,
    HandleWallSensorError,
    HandleSlamGlobalRelocation,
    Error,
    Shutdown,
    HandleCommunicateWithPlayerTimeoutError,
    TestATask,
    TestBTask
};

enum class NavStatus
{
    Normal = 0,
    Error
};

enum class CleanState
{
    StartUp,
    WallClean,
    RoomClean,
    BlockClean,
    CrossBlock,
    Completed
};

enum class CleanStatus
{
    UnCleaned=-1,
	UnCleanedForbiddenZone,
	ToBeCleaned,
	ToBeCleanedNearCleanedWall,
	ToBeCleanedWithWarning,
	ToBeCleanedForbiddenZone,
    
    Cleaned,
	CleanedWall,
	CleanedWallRealWallSide,
	CleanedWallBoundarySide,
	CleanedWallCompassSide,
	CleanedForbiddenZone
};

enum class MapStatus
{
    Occupied = 100,
    Unknow = -1,
    Free = 0
};

/*
enum class MapStatus
{
    Occupied = 100,
    Unknow = -1,
    Free = 0
};
*/
typedef  MapStatus GridStatus;   // for previou code compitable

enum class WallCleanState
{
    Init,
    FindingWall,
    GotoWall,
    WallFollowing,
    Completed,
    Exit
};
enum class WallFollowerState{
    StartUp,
    SearchWall,
    GoingToWall,
    FollowingWall,
    LaserFollowWall,
    WallSensorFollowWall,
    BigTurn,
    SharpCorner,
    ByPassObs,
    FollowingVirtualWall,
    FollowingCleanedBoundary, 
    Exit
};

enum class RoomCleanState
{
    Init,
    GotoNextLine,
    CleanCurrentLine,
    CleanObstacle,
    Completed,
    Exit
};

enum class BumperFlag{
    None_bumper,
    Left_bumper,
    Right_bumper,
    Both_bumper
};

enum class BlockIndex{
    DefaultIndex = 0,
};

/**/
float dtor(float deg);
float rtod(float rad);

template<typename  T> int sign(T& value) {
    if(value>(T)0)
        return 1;
    else if(fabs(value)<=1e-7)
        return 0;
    else
    {
        return -1;
    }
}


/* PID Definition */
class NavPid
{
public:
	/* data */
	float kp,ki,kd;
	float err, err_err, last_err;
	float ui, ud;
	float iMax, iMin;
	float output;
	float Min_ERR;
	float target;
	
public:
	float update(float tar, float fk);
	NavPid(float ekp, float eki, float ekd);
	void setPID(float ekp, float eki, float ekd);
    void setiMax(float imax);
	NavPid()
    {
        kp = 0;
        ki = 0;
        kd = 0;
        err = 0;
        err_err = 0;
        last_err = 0;
        ui = 0;
        ud = 0;
        iMax = 0;
        iMin = 0;
        output = 0;
        Min_ERR = 0;
        target = 0;
    };
	void reset();
	~NavPid();
};


class NavPath
{
private:
    /* data */
    std::vector<NavPoint> points;
public:
    NavPath(/* args */);
    ~NavPath();
    NavPoint pop();
    int size();
    void append(NavPoint in);
};

typedef struct Segment
{
    bool hasStPose;
    bool hasEdPose;
    bool constructForbidden;
    NavPose stPose;
    NavPose edPose;
    Segment()
    {
        hasStPose = false;
        hasEdPose = false;
        constructForbidden = false;
    }
    ~Segment()
    {
        
    }
    void setStPose(NavPose p)
    {
        stPose = p;
        hasStPose = true;
    }
    void setEdPose(NavPose p)
    {
        edPose = p;
        hasEdPose = true;
    }
    void reset()
    {
        Segment();
    }
    void setConstructForbidden(bool forbidden)
    {
        constructForbidden = forbidden;
    }
    float getSegmentLen()
    {
        return NavPoint(edPose.px - stPose.px, edPose.py - stPose.py).norm();
    }
}Segment_t;



float disBetweenNavPose(NavPose p1, NavPose p2);

OrientationState judgeLeftOrRight(NavPose curPose, NavPose judgePoint);


/* used in obsCleanTask */




#define PID_MAX_ERR_HISTORY_SIZE 10
typedef struct
{
	float kp;
	float ki;
	float kd;
	float err;
	float err_err;
	float last_err;
	float ui, ud;
	float ui_max, ui_min;
	float uMax, uMin;
	float output;
	float Min_ERR;
	float target;
	float fbk;
	float err_history[PID_MAX_ERR_HISTORY_SIZE];
	int err_history_cnt;
} pid_controller_t;

void pid_reset(pid_controller_t* pPID);
float pid_update(pid_controller_t* pPID, float target, float fbk, float ki_adjust_factor=1.0f);
void pid_init(pid_controller_t* pPID, float KP, float KI, float KD, float maxOutput, float minError);
float pid_update_with_D_window(pid_controller_t* pPID, float target, float fbk, float ki_adjust_factor = 1.0f, int D_window = 5);

#endif
