#include "HB_functions.h"

#define dt      0.002
#define     DSP                 0
#define     SSP                 2
#define     foot_up_ratio       0.4

Point pp0;

// A utility function to find next to top in a stack
Point nextToTop(stack<Point> &S)
{
    Point p = S.top();
    S.pop();
    Point res = S.top();
    S.push(p);
    return res;
}

// A utility function to swap two points
int swap(Point &p1, Point &p2)
{
    Point temp = p1;
    p1 = p2;
    p2 = temp;
}

// A utility function to return square of distance
// between p1 and p2
double distSq(Point p1, Point p2)
{
    return (p1.x - p2.x)*(p1.x - p2.x) +
          (p1.y - p2.y)*(p1.y - p2.y);
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    double val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

// check wheather point r is on the left side or right side of vector pq ( q - p )
// 0 ---> r is on the line of vector pq
// 1 ---> r is on the left side of vector pq
// 2 ---> r is on the right side of vector pq
int left_or_right(Point p, Point q, Point r)
{
    double val = (q.x - p.x)*(r.y - p.y) - (q.y - p.y)*(r.x - p.x);

    if (val == 0) return 0;
    return (val > 0) ? 1:-1;
}

// A function used by library function qsort() to sort an array of
// points with respect to the first point
int compare(const void *vp1, const void *vp2)
{
   Point *p1 = (Point *)vp1;
   Point *p2 = (Point *)vp2;

   // Find orientation
   int o = orientation(pp0, *p1, *p2);
   if (o == 0)
     return (distSq(pp0, *p2) >= distSq(pp0, *p1))? -1 : 1;

   return (o == 2)? -1: 1;
}

// Prints convex hull of a set of n points.
stack<Point> convexHull(Point points[], int n, int& n_final)
{
   // Find the bottommost point
   double ymin = points[0].y;
   int min = 0;
   for (int i = 1; i < n; i++)
   {
     double y = points[i].y;

     // Pick the bottom-most or chose the left
     // most point in case of tie
     if ((y < ymin) || ((ymin >= y - 0.0001 && ymin <= y + 0.0001)&& points[i].x < points[min].x))
        ymin = points[i].y, min = i;
   }

   // Place the bottom-most point at first position
   swap(points[0], points[min]);

   // Sort n-1 points with respect to the first point.
   // A point p1 comes before p2 in sorted ouput if p2
   // has larger polar angle (in counterclockwise
   // direction) than p1
   pp0 = points[0];
   qsort(&points[1], n-1, sizeof(Point), compare);

   // If two or more points make same angle with p0,
   // Remove all but the one that is farthest from p0
   // Remember that, in above sorting, our criteria was
   // to keep the farthest point at the end when more than
   // one points have same angle.
   int m = 1; // Initialize size of modified array
   for (int i=1; i<n; i++)
   {
       // Keep removing i while angle of i and i+1 is same
       // with respect to p0
       while (i < n-1 && orientation(pp0, points[i],
                                    points[i+1]) == 0)
          i++;


       points[m] = points[i];
       m++;  // Update size of modified array
   }

   // If modified array of points has less than 3 points,
   // convex hull is not possible
   stack<Point> SS;
   Point ss = {0,0};
   SS.push(ss);
   SS.push(ss);
   n_final = m;
   if (m < 3) return SS;

   // Create an empty stack and push first three points
   // to it.
   stack<Point> S;
   S.push(points[0]);
   S.push(points[1]);
   S.push(points[2]);

   // Process remaining n-3 points
   for (int i = 3; i < m; i++)
   {
      // Keep removing top while the angle formed by
      // points next-to-top, top, and points[i] makes
      // a non-left turn
      while (orientation(nextToTop(S), S.top(), points[i]) != 2)
         S.pop();
      S.push(points[i]);
   }

   // Now stack has the output points, print contents of stack
//   while (!S.empty())
//   {
//       Point p = S.top();
//       cout << "(" << p.x << ", " << p.y <<")" << endl;
//       S.pop();
//   }
   typename stack<Point>::size_type nn = S.size();

   n_final = nn;

   return S;
}

// Driver program to test above functions
//int main()
//{
//    Point points[] = {{0, 3}, {1, 1}, {2, 2}, {4, 4},
//                      {0, 0}, {1, 2}, {3, 1}, {3, 3}};
//    int n = sizeof(points)/sizeof(points[0]);
//    convexHull(points, n);
//    return 0;
//}

bool zmpInOutCheck(Point zmp, stack<Point> SP)
{
    int evaluation = 0;
    typename stack<Point>::size_type n = SP.size();

    Point *points = (Point*)malloc(sizeof(Point)*n);

    for(int i=0; i<n ; i++){
        Point p = SP.top();
        points[i] = p;
        //cout<<points[i].x<<", "<<points[i].y<<endl;
        SP.pop();
    }

    for(int i=0; i<n-1 ; i++){
        evaluation += left_or_right(points[i], points[i+1], zmp);
    }

    evaluation += left_or_right(points[n-1], points[0], zmp);
    //cout<<"n: "<<n<<"  eval:"<<evaluation<<endl;


    free(points);

    if(evaluation == n || evaluation == -n) return true;
    else return false;
}

int compare_double(const void *x, const void *y){
    if(*(double *)x > *(double *)y)
        return 1;
    else if(*(double *)x < *(double *)y)
        return -1;
    else
        return 0;
}

vec3 zmpProjectionToSP(vec3 _zmp, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF)
{

    //for hubo2 original foot
//    vec3 LF_lt1 = vec3(0.14, 0.08, 0);
//    vec3 LF_rt2 = vec3(0.14, -0.07, 0);
//    vec3 LF_rb3 = vec3(-0.08, -0.07, 0);
//    vec3 LF_lb4 = vec3(-0.08, 0.08, 0);

//    vec3 RF_lt1 = vec3(0.14, 0.07, 0);
//    vec3 RF_rt2 = vec3(0.14, -0.08, 0);
//    vec3 RF_rb3 = vec3(-0.08, -0.08, 0);
//    vec3 RF_lb4 = vec3(-0.08, 0.07, 0);

    //For Hyobin's new foot
    vec3 LF_lt1 = vec3((FootLength-0.005)/2, (FootWidth-0.005)/2, 0);
    vec3 LF_rt2 = vec3((FootLength-0.005)/2, -(FootWidth-0.005)/2, 0);
    vec3 LF_rb3 = vec3(-(FootLength-0.005)/2, -(FootWidth-0.005)/2, 0);
    vec3 LF_lb4 = vec3(-(FootLength-0.005)/2, (FootWidth-0.005)/2, 0);

    vec3 RF_lt1 = vec3((FootLength-0.005)/2, (FootWidth-0.005)/2, 0);
    vec3 RF_rt2 = vec3((FootLength-0.005)/2, -(FootWidth-0.005)/2, 0);
    vec3 RF_rb3 = vec3(-(FootLength-0.005)/2, -(FootWidth-0.005)/2, 0);
    vec3 RF_lb4 = vec3(-(FootLength-0.005)/2, (FootWidth-0.005)/2, 0);

    LF_lt1 = LF_lt1*mat3(_qLF);
    LF_rt2 = LF_rt2*mat3(_qLF);
    LF_rb3 = LF_rb3*mat3(_qLF);
    LF_lb4 = LF_lb4*mat3(_qLF);

    RF_lt1 = RF_lt1*mat3(_qRF);
    RF_rt2 = RF_rt2*mat3(_qRF);
    RF_rb3 = RF_rb3*mat3(_qRF);
    RF_lb4 = RF_lb4*mat3(_qRF);

    LF_lt1 = LF_lt1 + _pLF;
    LF_rt2 = LF_rt2 + _pLF;
    LF_rb3 = LF_rb3 + _pLF;
    LF_lb4 = LF_lb4 + _pLF;

    RF_lt1 = RF_lt1 + _pRF;
    RF_rt2 = RF_rt2 + _pRF;
    RF_rb3 = RF_rb3 + _pRF;
    RF_lb4 = RF_lb4 + _pRF;


    if(_F_RF.z > 50 && _F_LF.z > 50) // DSP phase
    {


        Point points[] = {{LF_lt1.x, LF_lt1.y}, {LF_rt2.x, LF_rt2.y}, {LF_rb3.x, LF_rb3.y}, {LF_lb4.x, LF_lb4.y},
                                {RF_lt1.x, RF_lt1.y}, {RF_rt2.x, RF_rt2.y}, {RF_rb3.x, RF_rb3.y}, {RF_lb4.x, RF_lb4.y}};

        int n_final = 100;
        stack<Point> SP = convexHull(points,8, n_final);

        Point zmp = {_zmp.x, _zmp.y};

        if(zmpInOutCheck(zmp, SP) == true){ // zmp is in the Support Polygon
            vec3 zmp = vec3(_zmp.x, _zmp.y,0);
            return zmp;
        }
        else{
            // zmp is out from the Support Polygon
            // we need to project to zmp into the Support Polygon
            Point SPs[n_final];
            Point zmp = {_zmp.x, _zmp.y};

            double centerX = 0,centerY = 0;

            for(int i=0; i<n_final ;i++){
                SPs[i] = SP.top();
                SP.pop();
                centerX += SPs[i].x;
                centerY += SPs[i].y;
                //cout<<"convex hull: "<<i<<" ("<<SPs[i].x<<", "<<SPs[i].y<<")"<<endl;
            }
            Point center = {centerX/n_final,centerY/n_final};

            int first_dist_index=0, second_dist_index=1;

            for(int i=0; i<n_final ; i++){
                int j = i+1;
                if(j == n_final) j = 0;
                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) == 0){
                    i++;
                    int j = i+1;
                    if(j == n_final) j = 0;
                }
                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

                int numOfsp=100;

                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

                if(numOfsp == 4){
                    if(zmpInOutCheck(cross_point,littleSP) == true){
                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
                            first_dist_index = i;
                            second_dist_index = j;
                        }
                        else{
                            first_dist_index = j;
                            second_dist_index = i;
                        }
                    }
                }
            }


            Point sp1 = SPs[first_dist_index];
            Point sp2 = SPs[second_dist_index];

//            cout<<"first index: "<<first_dist_index<<"  x : "<<sp1.x<<" y: "<<sp1.y<<endl;
//            cout<<"second index: "<<second_dist_index<<"  x : "<<sp2.x<<" y: "<<sp2.y<<endl;

            double c_sqare = distSq(zmp, sp2); // long side of triangle
            double a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
            double b_sqare = distSq(zmp, sp1); // short side of triangle

            vec3 proj_zmp;

            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

                vec3 p2n_proj = n*(dot(p,n)/dot(n));

                proj_zmp = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
            }
            else{
                proj_zmp = vec3(sp1.x, sp1.y, 0);
            }

            return proj_zmp;
        }
    }
    else{  // SSP phase
        stack<Point> SP;
        Point points[4];

        if(_F_LF.z <= 50){ // RF ssp
            points[0] = {RF_lt1.x, RF_lt1.y};
            points[1] = {RF_rt2.x, RF_rt2.y};
            points[2] = {RF_rb3.x, RF_rb3.y};
            points[3] = {RF_lb4.x, RF_lb4.y};
            SP.push({RF_lt1.x, RF_lt1.y});
            SP.push({RF_rt2.x, RF_rt2.y});
            SP.push({RF_rb3.x, RF_rb3.y});
            SP.push({RF_lb4.x, RF_lb4.y});
        }

        if(_F_RF.z <= 50){  // LF ssp
            points[0] = {LF_lt1.x, LF_lt1.y};
            points[1] = {LF_rt2.x, LF_rt2.y};
            points[2] = {LF_rb3.x, LF_rb3.y};
            points[3] = {LF_lb4.x, LF_lb4.y};
            SP.push({LF_lt1.x, LF_lt1.y});
            SP.push({LF_rt2.x, LF_rt2.y});
            SP.push({LF_rb3.x, LF_rb3.y});
            SP.push({LF_lb4.x, LF_lb4.y});
        }

        Point zmp = {_zmp.x, _zmp.y};


        if(zmpInOutCheck(zmp, SP) == true){ // zmp is in the Support Polygon
            vec3 zmp = vec3(_zmp.x, _zmp.y,0);
            return zmp;
        }
        else{
            // zmp is out from the Support Polygon
            // we need to project to zmp into the Support Polygon
            Point SPs[4];
            Point zmp = {_zmp.x, _zmp.y};

            double centerX = 0,centerY = 0;

            for(int i=0; i<4 ;i++){
                SPs[i] = points[i];
                centerX += SPs[i].x;
                centerY += SPs[i].y;
                //cout<<"convex hull: "<<i<<" ("<<SPs[i].x<<", "<<SPs[i].y<<")"<<endl;
            }
            Point center = {centerX/4,centerY/4};

            int first_dist_index=0, second_dist_index=1;

            for(int i=0; i<4 ; i++){
                int j = i+1;
                if(j == 4) j = 0;
                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) < 0.00001 &&
                            ((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) > -0.00001){
                    i++;
                    int j = i+1;
                    if(j == 4) j = 0;
                }
                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

                int numOfsp=100;

                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

                if(numOfsp == 4){
                    if(zmpInOutCheck(cross_point,littleSP) == true){
                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
                            first_dist_index = i;
                            second_dist_index = j;
                        }
                        else{
                            first_dist_index = j;
                            second_dist_index = i;
                        }
                    }
                }
            }


            Point sp1 = SPs[first_dist_index];
            Point sp2 = SPs[second_dist_index];

            //cout<<"first index: "<<first_dist_index<<"  x : "<<sp1.x<<" y: "<<sp1.y<<endl;
            //cout<<"second index: "<<second_dist_index<<"  x : _real_t_step"<<sp2.x<<" y: "<<sp2.y<<endl;

            double c_sqare = distSq(zmp, sp2); // long side of triangle
            double a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
            double b_sqare = distSq(zmp, sp1); // short side of triangle

            vec3 proj_zmp;

            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

                vec3 p2n_proj = n*(dot(p,n)/dot(n));

                proj_zmp = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
            }
            else{
                proj_zmp = vec3(sp1.x, sp1.y, 0);
            }

            return proj_zmp;
        }
    }
}

vec3 zmpProjectionToSP_large(vec3 _zmp, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF, double _offset)
{
//    //For hubo2 original foot
//    vec3 LF_lt1 = vec3(0.14+_offset, 0.08+_offset, 0);
//    vec3 LF_rt2 = vec3(0.14+_offset, -(0.07+_offset), 0);
//    vec3 LF_rb3 = vec3(-(0.08+_offset), -(0.07+_offset), 0);
//    vec3 LF_lb4 = vec3(-(0.08+_offset), 0.08+_offset, 0);

//    vec3 RF_lt1 = vec3(0.14+_offset, 0.07+_offset, 0);
//    vec3 RF_rt2 = vec3(0.14+_offset, -(0.08+_offset), 0);
//    vec3 RF_rb3 = vec3(-(0.08+_offset), -(0.08+_offset), 0);
//    vec3 RF_lb4 = vec3(-(0.08+_offset), 0.07+_offset, 0);

    //Fot Hyobin's foot
    vec3 LF_lt1 = vec3((FootLength+_offset)/2, (FootWidth+_offset)/2, 0);
    vec3 LF_rt2 = vec3((FootLength+_offset)/2, -(FootWidth+_offset)/2, 0);
    vec3 LF_rb3 = vec3(-(FootLength+_offset)/2, -(FootWidth+_offset)/2, 0);
    vec3 LF_lb4 = vec3(-(FootLength+_offset)/2, (FootWidth+_offset)/2, 0);

    vec3 RF_lt1 = vec3((FootLength+_offset)/2, (FootWidth+_offset)/2, 0);
    vec3 RF_rt2 = vec3((FootLength+_offset)/2, -(FootWidth+_offset)/2, 0);
    vec3 RF_rb3 = vec3(-(FootLength+_offset)/2, -(FootWidth+_offset)/2, 0);
    vec3 RF_lb4 = vec3(-(FootLength+_offset)/2, (FootWidth+_offset)/2, 0);

    LF_lt1 = LF_lt1*mat3(_qLF);
    LF_rt2 = LF_rt2*mat3(_qLF);
    LF_rb3 = LF_rb3*mat3(_qLF);
    LF_lb4 = LF_lb4*mat3(_qLF);

    RF_lt1 = RF_lt1*mat3(_qRF);
    RF_rt2 = RF_rt2*mat3(_qRF);
    RF_rb3 = RF_rb3*mat3(_qRF);
    RF_lb4 = RF_lb4*mat3(_qRF);

    LF_lt1 = LF_lt1 + _pLF;
    LF_rt2 = LF_rt2 + _pLF;
    LF_rb3 = LF_rb3 + _pLF;
    LF_lb4 = LF_lb4 + _pLF;

    RF_lt1 = RF_lt1 + _pRF;
    RF_rt2 = RF_rt2 + _pRF;
    RF_rb3 = RF_rb3 + _pRF;
    RF_lb4 = RF_lb4 + _pRF;


    if(_F_RF.z > 50 && _F_LF.z > 50) // DSP phase
    {


        Point points[] = {{LF_lt1.x, LF_lt1.y}, {LF_rt2.x, LF_rt2.y}, {LF_rb3.x, LF_rb3.y}, {LF_lb4.x, LF_lb4.y},
                                {RF_lt1.x, RF_lt1.y}, {RF_rt2.x, RF_rt2.y}, {RF_rb3.x, RF_rb3.y}, {RF_lb4.x, RF_lb4.y}};

        int n_final = 100;
        stack<Point> SP = convexHull(points,8, n_final);

        Point zmp = {_zmp.x, _zmp.y};

        if(zmpInOutCheck(zmp, SP) == true){ // zmp is in the Support Polygon
            vec3 zmp = vec3(_zmp.x, _zmp.y,0);
            return zmp;
        }
        else{
            // zmp is out from the Support Polygon
            // we need to project to zmp into the Support Polygon
            Point SPs[n_final];
            Point zmp = {_zmp.x, _zmp.y};

            double centerX = 0,centerY = 0;

            for(int i=0; i<n_final ;i++){
                SPs[i] = SP.top();
                SP.pop();
                centerX += SPs[i].x;
                centerY += SPs[i].y;
                //cout<<"convex hull: "<<i<<" ("<<SPs[i].x<<", "<<SPs[i].y<<")"<<endl;
            }
            Point center = {centerX/n_final,centerY/n_final};

            int first_dist_index=0, second_dist_index=1;

            for(int i=0; i<n_final ; i++){
                int j = i+1;
                if(j == n_final) j = 0;
                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) == 0){
                    i++;
                    int j = i+1;
                    if(j == n_final) j = 0;
                }
                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

                int numOfsp=100;

                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

                if(numOfsp == 4){
                    if(zmpInOutCheck(cross_point,littleSP) == true){
                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
                            first_dist_index = i;
                            second_dist_index = j;
                        }
                        else{
                            first_dist_index = j;
                            second_dist_index = i;
                        }
                    }
                }
            }


            Point sp1 = SPs[first_dist_index];
            Point sp2 = SPs[second_dist_index];

//            cout<<"first index: "<<first_dist_index<<"  x : "<<sp1.x<<" y: "<<sp1.y<<endl;
//            cout<<"second index: "<<second_dist_index<<"  x : "<<sp2.x<<" y: "<<sp2.y<<endl;

            double c_sqare = distSq(zmp, sp2); // long side of triangle
            double a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
            double b_sqare = distSq(zmp, sp1); // short side of triangle

            vec3 proj_zmp;

            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

                vec3 p2n_proj = n*(dot(p,n)/dot(n));

                proj_zmp = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
            }
            else{
                proj_zmp = vec3(sp1.x, sp1.y, 0);
            }

            return proj_zmp;
        }
    }
    else{  // SSP phase
        stack<Point> SP;
        Point points[4];

        if(_F_LF.z <= 50){ // RF ssp
            points[0] = {RF_lt1.x, RF_lt1.y};
            points[1] = {RF_rt2.x, RF_rt2.y};
            points[2] = {RF_rb3.x, RF_rb3.y};
            points[3] = {RF_lb4.x, RF_lb4.y};
            SP.push({RF_lt1.x, RF_lt1.y});
            SP.push({RF_rt2.x, RF_rt2.y});
            SP.push({RF_rb3.x, RF_rb3.y});
            SP.push({RF_lb4.x, RF_lb4.y});
        }

        if(_F_RF.z <= 50){  // LF ssp
            points[0] = {LF_lt1.x, LF_lt1.y};
            points[1] = {LF_rt2.x, LF_rt2.y};
            points[2] = {LF_rb3.x, LF_rb3.y};
            points[3] = {LF_lb4.x, LF_lb4.y};
            SP.push({LF_lt1.x, LF_lt1.y});
            SP.push({LF_rt2.x, LF_rt2.y});
            SP.push({LF_rb3.x, LF_rb3.y});
            SP.push({LF_lb4.x, LF_lb4.y});
        }

        Point zmp = {_zmp.x, _zmp.y};


        if(zmpInOutCheck(zmp, SP) == true){ // zmp is in the Support Polygon
            vec3 zmp = vec3(_zmp.x, _zmp.y,0);
            return zmp;
        }
        else{
            // zmp is out from the Support Polygon
            // we need to project to zmp into the Support Polygon
            Point SPs[4];
            Point zmp = {_zmp.x, _zmp.y};

            double centerX = 0,centerY = 0;

            for(int i=0; i<4 ;i++){
                SPs[i] = points[i];
                centerX += SPs[i].x;
                centerY += SPs[i].y;
                //cout<<"convex hull: "<<i<<" ("<<SPs[i].x<<", "<<SPs[i].y<<")"<<endl;
            }
            Point center = {centerX/4,centerY/4};

            int first_dist_index=0, second_dist_index=1;

            for(int i=0; i<4 ; i++){
                int j = i+1;
                if(j == 4) j = 0;
                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) < 0.00001 &&
                            ((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) > -0.00001){
                    i++;
                    int j = i+1;
                    if(j == 4) j = 0;
                }
                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

                int numOfsp=100;

                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

                if(numOfsp == 4){
                    if(zmpInOutCheck(cross_point,littleSP) == true){
                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
                            first_dist_index = i;
                            second_dist_index = j;
                        }
                        else{
                            first_dist_index = j;
                            second_dist_index = i;
                        }
                    }
                }
            }


            Point sp1 = SPs[first_dist_index];
            Point sp2 = SPs[second_dist_index];

            //cout<<"first index: "<<first_dist_index<<"  x : "<<sp1.x<<" y: "<<sp1.y<<endl;
            //cout<<"second index: "<<second_dist_index<<"  x : _real_t_step"<<sp2.x<<" y: "<<sp2.y<<endl;

            double c_sqare = distSq(zmp, sp2); // long side of triangle
            double a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
            double b_sqare = distSq(zmp, sp1); // short side of triangle

            vec3 proj_zmp;

            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

                vec3 p2n_proj = n*(dot(p,n)/dot(n));

                proj_zmp = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
            }
            else{
                proj_zmp = vec3(sp1.x, sp1.y, 0);
            }

            return proj_zmp;
        }
    }
}

vec3 zmpProjectionToSP_offset(vec3 _zmp, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF, double _Xoffset, double _Yoffset)
{
//    //For hubo2 original foot
//    vec3 LF_lt1 = vec3(0.14+_offset, 0.08+_offset, 0);
//    vec3 LF_rt2 = vec3(0.14+_offset, -(0.07+_offset), 0);
//    vec3 LF_rb3 = vec3(-(0.08+_offset), -(0.07+_offset), 0);
//    vec3 LF_lb4 = vec3(-(0.08+_offset), 0.08+_offset, 0);

//    vec3 RF_lt1 = vec3(0.14+_offset, 0.07+_offset, 0);
//    vec3 RF_rt2 = vec3(0.14+_offset, -(0.08+_offset), 0);
//    vec3 RF_rb3 = vec3(-(0.08+_offset), -(0.08+_offset), 0);
//    vec3 RF_lb4 = vec3(-(0.08+_offset), 0.07+_offset, 0);

    //temp foot (DRF Hubo Foot)
//    vec3 LF_lt1 = vec3(0.12+_Xoffset, 0.09+_Yoffset, 0);
//    vec3 LF_rt2 = vec3(0.12+_Xoffset, -(0.07+_Yoffset), 0);
//    vec3 LF_rb3 = vec3(-(0.12+_Xoffset), -(0.07+_Yoffset), 0);
//    vec3 LF_lb4 = vec3(-(0.12+_Xoffset), 0.09+_Yoffset, 0);

//    vec3 RF_lt1 = vec3(0.12+_Xoffset, 0.07+_Yoffset, 0);
//    vec3 RF_rt2 = vec3(0.12+_Xoffset, -(0.09+_Yoffset), 0);
//    vec3 RF_rb3 = vec3(-(0.12+_Xoffset), -(0.09+_Yoffset), 0);
//    vec3 RF_lb4 = vec3(-(0.12+_Xoffset), 0.07+_Yoffset, 0);

    //Fot Hyobin's foot
    vec3 LF_lt1 = vec3((FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);
    vec3 LF_rt2 = vec3((FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 LF_rb3 = vec3(-(FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 LF_lb4 = vec3(-(FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);

    vec3 RF_lt1 = vec3((FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);
    vec3 RF_rt2 = vec3((FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 RF_rb3 = vec3(-(FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 RF_lb4 = vec3(-(FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);

    LF_lt1 = mat3(_qLF)*LF_lt1;
    LF_rt2 = mat3(_qLF)*LF_rt2;
    LF_rb3 = mat3(_qLF)*LF_rb3;
    LF_lb4 = mat3(_qLF)*LF_lb4;

    RF_lt1 = mat3(_qRF)*RF_lt1;
    RF_rt2 = mat3(_qRF)*RF_rt2;
    RF_rb3 = mat3(_qRF)*RF_rb3;
    RF_lb4 = mat3(_qRF)*RF_lb4;

    LF_lt1 = LF_lt1 + _pLF;
    LF_rt2 = LF_rt2 + _pLF;
    LF_rb3 = LF_rb3 + _pLF;
    LF_lb4 = LF_lb4 + _pLF;

    RF_lt1 = RF_lt1 + _pRF;
    RF_rt2 = RF_rt2 + _pRF;
    RF_rb3 = RF_rb3 + _pRF;
    RF_lb4 = RF_lb4 + _pRF;

    userData->M2G.valveMode = 70;

    if(_F_RF.z > 20 && _F_LF.z > 20) // DSP phase
    {


        Point points[] = {{LF_lt1.x, LF_lt1.y}, {LF_rt2.x, LF_rt2.y}, {LF_rb3.x, LF_rb3.y}, {LF_lb4.x, LF_lb4.y},
                                {RF_lt1.x, RF_lt1.y}, {RF_rt2.x, RF_rt2.y}, {RF_rb3.x, RF_rb3.y}, {RF_lb4.x, RF_lb4.y}};

        int n_final = 8;
        stack<Point> SP = convexHull(points,8, n_final);

        userData->M2G.valveMode = 701;

        Point zmp = {_zmp.x, _zmp.y};

        if(zmpInOutCheck(zmp, SP) == true){ // zmp is in the Support Polygon
            vec3 zmp = vec3(_zmp.x, _zmp.y,0);
            return zmp;

            userData->M2G.valveMode = 71;
        }
        else{
            // zmp is out from the Support Polygon
            // we need to project to zmp into the Support Polygon
            Point SPs[n_final];
            Point zmp = {_zmp.x, _zmp.y};

//            double centerX = 0,centerY = 0;

            for(int i=0; i<n_final ;i++){
                SPs[i] = SP.top();
                SP.pop();
//                centerX += SPs[i].x;
//                centerY += SPs[i].y;
                //cout<<"convex hull: "<<i<<" ("<<SPs[i].x<<", "<<SPs[i].y<<")"<<endl;
            }
            Point center = {(_pRF.x + _pLF.x)/2,(_pRF.y + _pLF.y)/2};

            userData->M2G.valveMode = 72;

            int first_dist_index=0, second_dist_index=1;

            for(int i=0; i<n_final ; i++){
                int j = i+1;
                if(j == n_final) j = 0;
                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) == 0){
                    i++;
                    int j = i+1;
                    if(j == n_final) j = 0;
                }
                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

                int numOfsp=100;

                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

                userData->M2G.valveMode = 73;

                if(numOfsp == 4){
                    if(zmpInOutCheck(cross_point,littleSP) == true){
                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
                            first_dist_index = i;
                            second_dist_index = j;
                        }
                        else{
                            first_dist_index = j;
                            second_dist_index = i;
                        }
                    }
                }
            }

            userData->M2G.valveMode = 80;

            Point sp1 = SPs[first_dist_index];
            Point sp2 = SPs[second_dist_index];

//            cout<<"first index: "<<first_dist_index<<"  x : "<<sp1.x<<" y: "<<sp1.y<<endl;
//            cout<<"second index: "<<second_dist_index<<"  x : "<<sp2.x<<" y: "<<sp2.y<<endl;

            double c_sqare = distSq(zmp, sp2); // long side of triangle
            double a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
            double b_sqare = distSq(zmp, sp1); // short side of triangle

            vec3 proj_zmp;

            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

                vec3 p2n_proj = n*(dot(p,n)/dot(n));

                proj_zmp = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
            }
            else{
                proj_zmp = vec3(sp1.x, sp1.y, 0);
            }

            return proj_zmp;
        }

        userData->M2G.valveMode = 90;
    }
    else{  // SSP phase
        stack<Point> SP;
        Point points[4];

        if(_F_LF.z <= 20){ // RF ssp
            points[0] = {RF_lt1.x, RF_lt1.y};
            points[1] = {RF_rt2.x, RF_rt2.y};
            points[2] = {RF_rb3.x, RF_rb3.y};
            points[3] = {RF_lb4.x, RF_lb4.y};
            SP.push({RF_lt1.x, RF_lt1.y});
            SP.push({RF_rt2.x, RF_rt2.y});
            SP.push({RF_rb3.x, RF_rb3.y});
            SP.push({RF_lb4.x, RF_lb4.y});
        }

        if(_F_RF.z <= 20){  // LF ssp
            points[0] = {LF_lt1.x, LF_lt1.y};
            points[1] = {LF_rt2.x, LF_rt2.y};
            points[2] = {LF_rb3.x, LF_rb3.y};
            points[3] = {LF_lb4.x, LF_lb4.y};
            SP.push({LF_lt1.x, LF_lt1.y});
            SP.push({LF_rt2.x, LF_rt2.y});
            SP.push({LF_rb3.x, LF_rb3.y});
            SP.push({LF_lb4.x, LF_lb4.y});
        }

        Point zmp = {_zmp.x, _zmp.y};

        userData->M2G.valveMode = 100;


        if(zmpInOutCheck(zmp, SP) == true){ // zmp is in the Support Polygon
            vec3 zmp = vec3(_zmp.x, _zmp.y,0);
            return zmp;
        }
        else{
            // zmp is out from the Support Polygon
            // we need to project to zmp into the Support Polygon
            Point SPs[4];
            Point zmp = {_zmp.x, _zmp.y};

            double centerX = 0,centerY = 0;

            for(int i=0; i<4 ;i++){
                SPs[i] = points[i];
                centerX += SPs[i].x;
                centerY += SPs[i].y;
                //cout<<"convex hull: "<<i<<" ("<<SPs[i].x<<", "<<SPs[i].y<<")"<<endl;
            }
            Point center = {centerX/4,centerY/4};

            int first_dist_index=0, second_dist_index=1;

            for(int i=0; i<4 ; i++){
                int j = i+1;
                if(j == 4) j = 0;
                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) < 0.00001 &&
                            ((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) > -0.00001){
                    i++;
                    int j = i+1;
                    if(j == 4) j = 0;
                }
                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

                int numOfsp=100;

                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

                if(numOfsp == 4){
                    if(zmpInOutCheck(cross_point,littleSP) == true){
                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
                            first_dist_index = i;
                            second_dist_index = j;
                        }
                        else{
                            first_dist_index = j;
                            second_dist_index = i;
                        }
                    }
                }
            }

            userData->M2G.valveMode = 110;

            Point sp1 = SPs[first_dist_index];
            Point sp2 = SPs[second_dist_index];

            //cout<<"first index: "<<first_dist_index<<"  x : "<<sp1.x<<" y: "<<sp1.y<<endl;
            //cout<<"second index: "<<second_dist_index<<"  x : _real_t_step"<<sp2.x<<" y: "<<sp2.y<<endl;

            double c_sqare = distSq(zmp, sp2); // long side of triangle
            double a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
            double b_sqare = distSq(zmp, sp1); // short side of triangle

            vec3 proj_zmp;

            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

                vec3 p2n_proj = n*(dot(p,n)/dot(n));

                proj_zmp = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
            }
            else{
                proj_zmp = vec3(sp1.x, sp1.y, 0);
            }

            return proj_zmp;
        }
    }
}

vec3 zmpProjectionTo_StanceFoot(int _swingFoot, vec3 _Vector_stanceFootFrame, double _Xoffset, double _Yoffset){
    vec3 Vector_Saturated_ = _Vector_stanceFootFrame;

    if(_swingFoot == -1){ // RFoot swing --> LF stance
        if(Vector_Saturated_.x > (FootLength+_Xoffset)/2) Vector_Saturated_.x = (FootLength+_Xoffset)/2;
        if(Vector_Saturated_.x < -(FootLength+_Xoffset)/2) Vector_Saturated_.x = -(FootLength+_Xoffset)/2;

//        if(Vector_Saturated_.y > 0.030) Vector_Saturated_.y = 0.030;
//        if(Vector_Saturated_.y < -(FootWidth+_Yoffset)/2) Vector_Saturated_.y = -(FootWidth+_Yoffset)/2;
        if(Vector_Saturated_.y > 0.027) Vector_Saturated_.y = 0.027;
        if(Vector_Saturated_.y < -0.027) Vector_Saturated_.y = -0.027;
    }
    else if(_swingFoot == 1){ // LFoot
        if(Vector_Saturated_.x > (FootLength+_Xoffset)/2) Vector_Saturated_.x = (FootLength+_Xoffset)/2;
        if(Vector_Saturated_.x < -(FootLength+_Xoffset)/2) Vector_Saturated_.x = -(FootLength+_Xoffset)/2;

//        if(Vector_Saturated_.y > (FootWidth+_Yoffset)/2) Vector_Saturated_.y = (FootWidth+_Yoffset)/2;
//        if(Vector_Saturated_.y < -0.030) Vector_Saturated_.y = -0.030;
        if(Vector_Saturated_.y > 0.027) Vector_Saturated_.y = 0.027;
        if(Vector_Saturated_.y < -0.027) Vector_Saturated_.y = -0.027;
    }
    else{
        if(Vector_Saturated_.x > (FootLength+_Xoffset)/2) Vector_Saturated_.x = (FootLength+_Xoffset)/2;
        if(Vector_Saturated_.x < -(FootLength+_Xoffset)/2) Vector_Saturated_.x = -(FootLength+_Xoffset)/2;

        if(Vector_Saturated_.y > (FootWidth+_Yoffset)/2) Vector_Saturated_.y = (FootWidth+_Yoffset)/2;
        if(Vector_Saturated_.y < -(FootWidth+_Yoffset)/2) Vector_Saturated_.y = -(FootWidth+_Yoffset)/2;
    }
            
    // Local vector saturation in to the foot


    
    

    return Vector_Saturated_;
}


vec3 ZMP_calc_global(vec3 _pRF, quat _qRF, vec3 _F_RF, vec3 _M_RF, vec3 _pLF, quat _qLF, vec3 _F_LF,vec3 _M_LF)
{
    // <Ouput> : global zmp    <Input> : global position of foot, local Force/Torque, orientation of foot
    if(_F_RF.z <= 0) _F_RF.z = 0;
    if(_F_LF.z <= 0) _F_LF.z = 0;
    
    if(_F_RF.z < 2.0 && _F_LF.z<2.0)
    {
        vec3 ZMP_global;
        ZMP_global = _pRF*0.5+_pLF*0.5;
        return ZMP_global;//not good...
    }

    vec3 pRF_real = _pRF + vec3(0,0,0.045);
    vec3 pLF_real = _pLF + vec3(0,0,0.045);

    // transform local force and moment to global frame
    mat3 RF_TFmat = mat3(_qRF);
    vec3 F_RF_global = RF_TFmat*_F_RF;
    vec3 M_RF_global = RF_TFmat*_M_RF;

    mat3 LF_TFmat = mat3(_qLF);
    vec3 F_LF_global = LF_TFmat*_F_LF;
    vec3 M_LF_global = LF_TFmat*_M_LF;

    vec3 rxF_LF_global = cross(pLF_real,F_LF_global);
    vec3 rxF_RF_global = cross(pRF_real,F_RF_global);

    vec3 ZMP_global;
    ZMP_global.x = (-rxF_LF_global.y - rxF_RF_global.y - M_LF_global.y - M_RF_global.y)/(F_LF_global.z + F_RF_global.z);
    ZMP_global.y = (rxF_LF_global.x + rxF_RF_global.x + M_LF_global.x + M_RF_global.x)/(F_LF_global.z + F_RF_global.z);
    ZMP_global.z = (_pRF.z*F_RF_global.z + _pLF.z*F_LF_global.z)/(F_RF_global.z + F_LF_global.z);

    return ZMP_global;
}

vec3 Calc_local(vec3 _pRF, quat _qRF, vec3 _pLF, quat _qLF, vec3 _pGlobal){
    // Position(such as ZMP) calculation with respect to Robot Local frame ( Orgin : meddle of foot, Orientation : average of foot

    vec3 pCenter; // middle of two foot
    quat qCenter; // average orientation of two foot
    vec3 _p_local;

    // Foot Center in Global Coord
    pCenter = (_pRF + _pLF)/2;
    pCenter.z = 0;

    // Foot average Orientaion in Global Coord
    quat qRF_inv = inverse_HB(_qRF);
    quat delta_quat = qRF_inv*_qLF;
    double delta_angle;
    vec3 delta_axis;
    axisAngle(delta_quat, delta_angle, delta_axis);  //quat --> angle and vector expression

    if(delta_angle <= PI){
        delta_angle /= 2;
    }
    else{
        delta_angle = 2*PI - delta_angle;
        delta_angle /= 2;
    }

    delta_quat.w = cos(delta_angle/2.0);

    qCenter = _qRF*delta_quat;

    //ZMP_global = ZMP_calc_global(pRF,qRF,F_RF,M_RF,pLF,qLF,F_LF,M_LF);
    _p_local = _pGlobal - pCenter;
    quat inv_qCenter = inverse_HB(qCenter);

    _p_local = mat3(inv_qCenter)*_p_local;

    return _p_local;
}


vec3 global2local_vec(quat _qRF, quat _qLF, vec3 _VecGlobal){
//    // Calculate global vector to local vector
//    vec3 _VecLocal;
//    quat qCenter;

//    // Foot average Orientaion in Global Coord
//    quat qRF_inv = inverse(_qRF);
//    quat delta_quat = qRF_inv*_qLF;
//    double delta_angle;
//    vec3 delta_axis;
//    axisAngle(delta_quat, delta_angle, delta_axis);  //quat --> angle and vector expression

//    if(delta_angle <= PI){
//        delta_angle /= 2;
//    }
//    else{
//        delta_angle = 2*PI - delta_angle;
//        delta_angle /= 2;
//    }

//    delta_quat.w = cos(delta_angle/2.0);

//    qCenter = _qRF*delta_quat;

//    quat inv_qCenter = inverse(qCenter);

//    _VecLocal = inv_qCenter*_VecGlobal;

//    return _VecLocal;

    //// Foot average Orientation in global Frame
    // find "g_R_local" matrix---------------------------------------------------------------
    rpy Euler_angle_RF_rad = rpy(_qRF);  //rpy print out angle between -180 ~ 180;
    rpy Euler_angle_LF_rad = rpy(_qLF);

    quat RF_yaw_quat = quat(vec3(0,0,1),Euler_angle_RF_rad.y);
    quat LF_yaw_quat = quat(vec3(0,0,1),Euler_angle_LF_rad.y);

//    cout<<"RF_yaw_quat : "<<endl;
//    cout<<RF_yaw_quat.w<<", "<<RF_yaw_quat.x<<", "<<RF_yaw_quat.y<<", "<<RF_yaw_quat.z<<endl;


//    cout<<"inverse(RF_yaw_quat) : "<<endl;
//    cout<<inverse(RF_yaw_quat).w<<", "<<inverse(RF_yaw_quat).x<<", "<<inverse(RF_yaw_quat).y<<", "<<inverse(RF_yaw_quat).z<<endl<<endl;

    quat delta_quat = inverse_HB(RF_yaw_quat)*LF_yaw_quat;

    rpy delta_rpy = rpy(delta_quat);

    delta_rpy.y = delta_rpy.y/2;

    mat3 g_R_local = mat3(RF_yaw_quat*quat(delta_rpy));
    //--------------------------------------------------------------------------------------

    mat3 local_R_g = inverse_HB(g_R_local);


//    cout<<"g_R_local : "<<endl;
//    cout<<g_R_local.m00<<", "<<g_R_local.m01<<", "<<g_R_local.m02<<endl;
//    cout<<g_R_local.m10<<", "<<g_R_local.m11<<", "<<g_R_local.m12<<endl;
//    cout<<g_R_local.m20<<", "<<g_R_local.m21<<", "<<g_R_local.m22<<endl;

//    cout<<"local_R_g : "<<endl;
//    cout<<local_R_g.m00<<", "<<local_R_g.m01<<", "<<local_R_g.m02<<endl;
//    cout<<local_R_g.m10<<", "<<local_R_g.m11<<", "<<local_R_g.m12<<endl;
//    cout<<local_R_g.m20<<", "<<local_R_g.m21<<", "<<local_R_g.m22<<endl<<endl;


    vec3 VecLocal_ = local_R_g*_VecGlobal;

    return VecLocal_;

}

vec3 global2local_point(quat _qRF, quat _qLF, vec3 _pRF, vec3 _pLF, vec3 _pGlobal){
    // Position(such as ZMP) calculation with respect to Robot Local frame ( Orgin : meddle of foot, Orientation : average of foot )

//    vec3 pCenter; // middle of two foot
//    quat qCenter; // average orientation of two foot
//    vec3 _p_local;

//    // Foot Center in Global Coord
//    pCenter = (_pRF + _pLF)/2;
//    pCenter.z = 0;

//    // Foot average Orientaion in Global Coord
//    quat qRF_inv = inverse(_qRF);
//    quat delta_quat = qRF_inv*_qLF;
//    double delta_angle;
//    vec3 delta_axis;
//    axisAngle(delta_quat, delta_angle, delta_axis);  //quat --> angle and vector expression

//    if(delta_angle <= PI){
//        delta_angle /= 2;
//    }
//    else{
//        delta_angle = 2*PI - delta_angle;
//        delta_angle /= 2;
//    }

//    delta_quat.w = cos(delta_angle/2.0);

//    qCenter = _qRF*delta_quat;

//    //ZMP_global = ZMP_calc_global(pRF,qRF,F_RF,M_RF,pLF,qLF,F_LF,M_LF);
//    _p_local = _pGlobal - pCenter;
//    quat inv_qCenter = inverse(qCenter);

//    _p_local = mat3(inv_qCenter)*_p_local;

//    return _p_local;

    //// Position(such as ZMP) calculation with respect to Robot Local frame ( Orgin : middle of foot, Orientation : average of foot )

    vec3 pCenter; // middle of two foot

    // Foot Center in Global Coord
    pCenter = (_pRF + _pLF)/2;
    pCenter.z = 0;

    // Foot average Orientation in global Frame
    // find "g_R_local" matrix---------------------------------------------------------------
    rpy Euler_angle_RF_rad = rpy(_qRF);  //rpy print out angle between -180 ~ 180;
    rpy Euler_angle_LF_rad = rpy(_qLF);

    quat RF_yaw_quat = quat(vec3(0,0,1),Euler_angle_RF_rad.y);
    quat LF_yaw_quat = quat(vec3(0,0,1),Euler_angle_LF_rad.y);

    quat delta_quat = inverse_HB(RF_yaw_quat)*LF_yaw_quat;

    rpy delta_rpy = rpy(delta_quat);

    delta_rpy.y = delta_rpy.y/2;

    mat3 g_R_local = mat3(RF_yaw_quat*quat(delta_rpy));
    //--------------------------------------------------------------------------------------

    mat3 local_R_g = inverse_HB(g_R_local);

    vec3 pCenter2point_global = _pGlobal - pCenter;

    vec3 pCenter2point_local = local_R_g*pCenter2point_global;

    return pCenter2point_local;

}

quat global2local_quat(quat _qRF, quat _qLF, quat _QuatGlobal){
//    // Calculate global vector to local vector
//    quat _QuatLocal;
//    quat qCenter;

//    // Foot average Orientaion in Global Coord
//    quat qRF_inv = inverse(_qRF);
//    quat delta_quat = qRF_inv*_qLF;
//    double delta_angle;
//    vec3 delta_axis;
//    axisAngle(delta_quat, delta_angle, delta_axis);  //quat --> angle and vector expression

//    if(delta_angle <= PI){
//        delta_angle /= 2;
//    }
//    else{
//        delta_angle = 2*PI - delta_angle;
//        delta_angle /= 2;
//    }

//    delta_quat.w = cos(delta_angle/2.0);

//    qCenter = _qRF*delta_quat;

//    quat inv_qCenter = inverse(qCenter);

//    _QuatLocal = inv_qCenter*_QuatGlobal;

//    return _QuatLocal;

    // find "g_R_local" matrix---------------------------------------------------------------
    rpy Euler_angle_RF_rad = rpy(_qRF);  //rpy print out angle between -180 ~ 180;
    rpy Euler_angle_LF_rad = rpy(_qLF);

    quat RF_yaw_quat = quat(vec3(0,0,1),Euler_angle_RF_rad.y);
    quat LF_yaw_quat = quat(vec3(0,0,1),Euler_angle_LF_rad.y);

    quat delta_quat = inverse_HB(RF_yaw_quat)*LF_yaw_quat;

    rpy delta_rpy = rpy(delta_quat);

    delta_rpy.y = delta_rpy.y/2;

    mat3 g_R_local = mat3(RF_yaw_quat*quat(delta_rpy));
    //--------------------------------------------------------------------------------------

    mat3 local_R_g = inverse_HB(g_R_local);

    quat QuatLocal_ = quat(local_R_g)*_QuatGlobal;

    return QuatLocal_;

}

vec3 local2global_vec(quat _qRF, quat _qLF, vec3 _VecLocal){
//    // Calculate local vector to global vector
//    vec3 _VecGlobal;
//    quat qCenter;

//    // Foot average Orientaion in Global Coord
//    quat qRF_inv = inverse(_qRF);
//    quat delta_quat = qRF_inv*_qLF;
//    double delta_angle;
//    vec3 delta_axis;
//    axisAngle(delta_quat, delta_angle, delta_axis);  //quat --> angle and vector expression

//    if(delta_angle <= PI){
//        delta_angle /= 2;
//    }
//    else{
//        delta_angle = 2*PI - delta_angle;
//        delta_angle /= 2;
//    }

//    delta_quat.w = cos(delta_angle/2.0);

//    qCenter = _qRF*delta_quat;

//    _VecGlobal = qCenter*_VecLocal;

//    return _VecGlobal;

    //// Calculate local vector to global vector
    // find "g_R_local" matrix---------------------------------------------------------------
    rpy Euler_angle_RF_rad = rpy(_qRF);  //rpy print out angle between -180 ~ 180;
    rpy Euler_angle_LF_rad = rpy(_qLF);

    quat RF_yaw_quat = quat(vec3(0,0,1),Euler_angle_RF_rad.y);
    quat LF_yaw_quat = quat(vec3(0,0,1),Euler_angle_LF_rad.y);

    quat delta_quat = inverse_HB(RF_yaw_quat)*LF_yaw_quat;

    rpy delta_rpy = rpy(delta_quat);

    delta_rpy.y = delta_rpy.y/2;

    mat3 g_R_local = mat3(RF_yaw_quat*quat(delta_rpy));
    //--------------------------------------------------------------------------------------

    vec3 VecGlobal_ = g_R_local*_VecLocal;

    return VecGlobal_;

}

vec3 local2global_point(quat _qRF, quat _qLF, vec3 _pRF, vec3 _pLF, vec3 _pLocal){

    //// get Center point of Feet
    vec3 pCenter; // middle of two foot

    // Foot Center in Global Coord
    pCenter = (_pRF + _pLF)/2;
    pCenter.z = 0;

    //// Calculate local point to global point
    // find "g_R_local" matrix---------------------------------------------------------------
    rpy Euler_angle_RF_rad = rpy(_qRF);  //rpy print out angle between -180 ~ 180;
    rpy Euler_angle_LF_rad = rpy(_qLF);

    quat RF_yaw_quat = quat(vec3(0,0,1),Euler_angle_RF_rad.y);
    quat LF_yaw_quat = quat(vec3(0,0,1),Euler_angle_LF_rad.y);

    quat delta_quat = inverse_HB(RF_yaw_quat)*LF_yaw_quat;

    rpy delta_rpy = rpy(delta_quat);

    delta_rpy.y = delta_rpy.y/2;

    mat3 g_R_local = mat3(RF_yaw_quat*quat(delta_rpy));
    //--------------------------------------------------------------------------------------


    vec3 pCenter2plocal_global = g_R_local*(_pLocal);

    vec3 pGlobal_ = pCenter + pCenter2plocal_global;

    return pGlobal_;

}

double get_Pel_yaw_from_qRF_qLF(quat _qRF, quat _qLF){
    //// Foot average Orientation in global Frame
    rpy Euler_angle_RF_rad = rpy(_qRF);  //rpy print out angle between -180 ~ 180;
    rpy Euler_angle_LF_rad = rpy(_qLF);

    quat RF_yaw_quat = quat(vec3(0,0,1),Euler_angle_RF_rad.y);
    quat LF_yaw_quat = quat(vec3(0,0,1),Euler_angle_LF_rad.y);

    quat delta_quat = inverse_HB(RF_yaw_quat)*LF_yaw_quat;

    rpy delta_rpy = rpy(delta_quat);

    delta_rpy.y = delta_rpy.y/2;

    rpy qPel_rpy = rpy(RF_yaw_quat*quat(delta_rpy));

    return qPel_rpy.y;
    //--------------------------------------------------------------------------------------
}



vec3 COM_measurment_by_quat(vec3 _COM, vec3 _ZMP_global, quat _IMUquat, quat _qPel){
    mat3 rot_pitroll_wrtpelv = mat3(_IMUquat);

    vec3 COM = _COM;  // local COM
    COM.z = 0.62;

    vec3 ZMP_global = _ZMP_global;
    ZMP_global.z = 0;



    // Measure Global COM position using FK and ZMP and Gyro
    mat3 rot_pel = mat3(_qPel);
    vec3 ZMP2COM = COM - ZMP_global;
    //cout<<"ZMP2COM: "<<ZMP2COM.z<<endl;

    vec3 COM_measure = ZMP_global + rot_pitroll_wrtpelv*ZMP2COM;




    return COM_measure;
}

vec3 dCOM_measurement_by_quat(vec3 _pCOMm, vec3 _udCOM, vec3 _ZMP_global, quat _IMUquat, vec3 _IMUomega){

    // Measure Global COM position using FK and ZMP and Gyro
    vec3 g_dpCOM_linear = mat3(_IMUquat)*_udCOM;

    vec3 g_dpCOM_angular = cross(_IMUomega, (_pCOMm - _ZMP_global));
    
    vec3 dCOM_measure = g_dpCOM_linear + g_dpCOM_angular;
    
    return dCOM_measure;
    
}

quat Global_qPel_measurment(vec3 _IMUangle, quat _qPel){
    mat3 rot_pitroll_wrtpelv = mat3(cos(_IMUangle.y*D2Rf),sin(_IMUangle.y*D2Rf)*sin(_IMUangle.x*D2Rf), sin(_IMUangle.y*D2Rf)*cos(_IMUangle.x*D2Rf),
                          0, cos(_IMUangle.x*D2Rf), -sin(_IMUangle.x*D2Rf),
                          -sin(_IMUangle.y*D2Rf), cos(_IMUangle.y*D2Rf)*sin(_IMUangle.x*D2Rf), cos(_IMUangle.y*D2Rf)*cos(_IMUangle.x*D2Rf));
    // Measure Global COM position using FK and ZMP and Gyro
    mat3 rot_pel = mat3(_qPel);

    return quat(rot_pel*rot_pitroll_wrtpelv);
}

vec3 FootZ_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _Z_dZ_ddZ_Foot_old, double _MaxFootUp, double _minFootDown){
    double t_up  = _real_t_step*(1-_dsp_ratio)*foot_up_ratio;
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;
    //double t_down = _real_t_step - t_up - 2*t_half_dsp;

    if(_t_foot_now <  t_half_dsp - 0.5*dt)
    {
    }
    else if(_t_foot_now < t_up + t_half_dsp - 0.5*dt){             //foot rising, 5th order trajectory up
//        double st_5,st_4,st_3,st_2,st_1;
//        st_1 = _t_foot_now;
//        st_2 = st_1*st_1;
//        st_3 = st_2*st_1;
//        st_4 = st_3*st_1;
//        st_5 = st_4*st_1;

//        VectorNd upParams = calc_5th(_t_foot_now - dt, t_up + t_half_dsp, _Z_dZ_ddZ_Foot_old, vec3(_MaxFootUp,0,0));
//        double zFoot = upParams(0)*st_5 + upParams(1)*st_4 + upParams(2)*st_3 + upParams(3)*st_2 + upParams(4)*st_1 + upParams(5);
//        double dzFoot = 5*upParams(0)*st_4 + 4*upParams(1)*st_3 + 3*upParams(2)*st_2 + 2*upParams(3)*st_1 + upParams(4);
//        double ddzFoot = 20*upParams(0)*st_3 + 12*upParams(1)*st_2 + 6*upParams(2)*st_1 + 2*upParams(3);

        //vec3 p_dp_ddp = calc_5th_GG(_t_foot_now - dt, t_up + t_half_dsp, _Z_dZ_ddZ_Foot_old, vec3(_MaxFootUp,0,0));
        vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, t_up + t_half_dsp, _Z_dZ_ddZ_Foot_old, vec3(_MaxFootUp,0,0));

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot); //return z, dz, ddz

    }
    else if(_t_foot_now < _real_t_step - t_half_dsp - 0.5*dt){                  // foot down, 5th order trajectory down
//        double st_5,st_4,st_3,st_2,st_1;
//        st_1 = _t_foot_now;
//        st_2 = st_1*st_1;
//        st_3 = st_2*st_1;
//        st_4 = st_3*st_1;
//        st_5 = st_4*st_1;

//        VectorNd dnParams = calc_5th(_t_foot_now - dt,  t_up + t_half_dsp + t_down + dt, _Z_dZ_ddZ_Foot_old, vec3(_minFootDown,0,0));
//        double zFoot = dnParams(0)*st_5 + dnParams(1)*st_4 + dnParams(2)*st_3 + dnParams(3)*st_2 + dnParams(4)*st_1 + dnParams(5);
//        double dzFoot = 5*dnParams(0)*st_4 + 4*dnParams(1)*st_3 + 3*dnParams(2)*st_2 + 2*dnParams(3)*st_1 + dnParams(4);
//        double ddzFoot = 20*dnParams(0)*st_3 + 12*dnParams(1)*st_2 + 6*dnParams(2)*st_1 + 2*dnParams(3);

        //vec3 p_dp_ddp = calc_5th_GG(_t_foot_now - dt, t_up + t_half_dsp + t_down + dt, _Z_dZ_ddZ_Foot_old, vec3(_minFootDown,0,0));
        vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, _real_t_step - t_half_dsp + dt, _Z_dZ_ddZ_Foot_old, vec3(_minFootDown,0,0));

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot);  //return z, dz, ddz
    }
    else
    {
    }

    return _Z_dZ_ddZ_Foot_old;
}

vec3 FootZ_Landing_trajectory(double _landing_time, double _t_foot_now, vec3 _Z_dZ_ddZ_Foot_old, double _minFootDown){

    // _landing_time : the time when foot landed

    //vec3 p_dp_ddp = calc_5th_GG(_t_foot_now - dt, _landing_time + dt, _Z_dZ_ddZ_Foot_old, vec3(_minFootDown,0,0));
    vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, _landing_time + dt, _Z_dZ_ddZ_Foot_old, vec3(_minFootDown,0,0));

    return p_dp_ddp;
}

vec3 FootXY_Landing_trajectory(double _landing_time, double _t_foot_now, vec3 _p_dp_ddp_Foot_old, double _goal){

    // _landing_time : the time when foot landed

    //vec3 p_dp_ddp = calc_5th_GG(_t_foot_now - dt, _landing_time + dt, _Z_dZ_ddZ_Foot_old, vec3(_minFootDown,0,0));
    vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, _landing_time + dt, _p_dp_ddp_Foot_old, vec3(_goal,0,0));

    return p_dp_ddp;
}

vec3 FootZ_recovary(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _Z_dZ_ddZ_Foot_old, double _last_landing_height){
    double t_up  = _real_t_step*foot_up_ratio*(1.0 - _dsp_ratio);
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;
    double t_down = _real_t_step - t_up - 2*t_half_dsp;

    double recover_height = 0.0;
    if(_last_landing_height > 0.03 || _last_landing_height < -0.03)
        recover_height = 0.0;//_last_landing_height/2;
    else
        recover_height = 0.0;

    if(_t_foot_now < t_half_dsp - 0.5*dt)
    {
    }
    else if( _t_foot_now < t_up + t_half_dsp + t_down - 0.5*dt){                  // foot down, 5th order trajectory down
//        double st_5,st_4,st_3,st_2,st_1;
//        st_1 = _t_foot_now;
//        st_2 = st_1*st_1;
//        st_3 = st_2*st_1;
//        st_4 = st_3*st_1;
//        st_5 = st_4*st_1;

//        VectorNd dnParams = calc_5th(_t_foot_now - dt,  t_up + t_half_dsp + t_down + dt, _Z_dZ_ddZ_Foot_old, vec3(recover_height,0,0));
//        double zFoot = dnParams(0)*st_5 + dnParams(1)*st_4 + dnParams(2)*st_3 + dnParams(3)*st_2 + dnParams(4)*st_1 + dnParams(5);
//        double dzFoot = 5*dnParams(0)*st_4 + 4*dnParams(1)*st_3 + 3*dnParams(2)*st_2 + 2*dnParams(3)*st_1 + dnParams(4);
//        double ddzFoot = 20*dnParams(0)*st_3 + 12*dnParams(1)*st_2 + 6*dnParams(2)*st_1 + 2*dnParams(3);

        vec3 p_dp_ddp = calc_5th_GG(_t_foot_now - dt,  t_up + t_half_dsp + t_down + dt, _Z_dZ_ddZ_Foot_old, vec3(recover_height,0,0));

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot);  //return z, dz, ddz
    }
    else
    {
    }

    return _Z_dZ_ddZ_Foot_old;
}

vec3 FootZ_recovary_trapezoidal(double _t_step, double _t_now, double _dsp_ratio, double _del_t, double _max_speed, vec3 _Z_dZ_ddZ_old){
    double t_up  = _t_step*foot_up_ratio*(1.0 - _dsp_ratio);
    double t_half_dsp = _t_step*_dsp_ratio/2.0;
    double t_down = _t_step - t_up - 2*t_half_dsp;

    double dz = 0;
    if(_max_speed == 0){
        return _Z_dZ_ddZ_old;
    }

    if(_t_now < t_half_dsp - 0.5*dt)
    {
    }
    else if( _t_now < t_up + t_half_dsp + t_down - 0.5*dt){
        dz = calc_trap(_del_t, t_up + t_down, _max_speed, _t_now - t_half_dsp);


        return vec3(_Z_dZ_ddZ_old[0]+dz*dt, dz, 0);
    }
    else
    {
    }

    //double calc_trap(double _del_t, double _te, double _he, double _t_now)

    return _Z_dZ_ddZ_old;



}

vec3 FootX_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _X_dX_ddX_Foot_old, double _X_footStep){
    double t_forward  = 0.9*_real_t_step*(1.0 - _dsp_ratio);
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;

    if(_t_foot_now < t_half_dsp - 0.5*dt)
    {
    }
    else if(_t_foot_now < t_forward + t_half_dsp - 0.5*dt)
    {             //foot rising, 5th order trajectory up
//        double st_5,st_4,st_3,st_2,st_1;
//        st_1 = _t_foot_now;
//        st_2 = st_1*st_1;
//        st_3 = st_2*st_1;
//        st_4 = st_3*st_1;
//        st_5 = st_4*st_1;

//        VectorNd Params = calc_5th(_t_foot_now - dt, t_forward + t_half_dsp, _X_dX_ddX_Foot_old, vec3(_X_footStep,0,0));
//        double xFoot = Params(0)*st_5 + Params(1)*st_4 + Params(2)*st_3 + Params(3)*st_2 + Params(4)*st_1 + Params(5);
//        double dxFoot = 5*Params(0)*st_4 + 4*Params(1)*st_3 + 3*Params(2)*st_2 + 2*Params(3)*st_1 + Params(4);
//        double ddxFoot = 20*Params(0)*st_3 + 12*Params(1)*st_2 + 6*Params(2)*st_1 + 2*Params(3);

        //vec3 p_dp_ddp = calc_5th_GG(_t_foot_now - dt, t_forward + t_half_dsp, _X_dX_ddX_Foot_old, vec3(_X_footStep,0,0));
        vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, t_forward + t_half_dsp, _X_dX_ddX_Foot_old, vec3(_X_footStep,0,0));

        return p_dp_ddp;//vec3(xFoot, dxFoot, ddxFoot); //return z, dz, ddz
    }
    else
    {
    }

    return _X_dX_ddX_Foot_old;
}

vec3 FootY_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _y_dy_ddy_Foot_old, double _Y_footStep){
    double t_moving = 1.0*_real_t_step*(1.0 - _dsp_ratio);
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;

    if(_t_foot_now < t_half_dsp - 0.5*dt)
    {
    }
    else if(_t_foot_now <  t_moving + t_half_dsp - 0.5*dt){             // 5th order trajectory up
        double st_5,st_4,st_3,st_2,st_1;
        st_1 = _t_foot_now;
        st_2 = st_1*st_1;
        st_3 = st_2*st_1;
        st_4 = st_3*st_1;
        st_5 = st_4*st_1;

        VectorNd Params = calc_5th(_t_foot_now - dt, t_moving + t_half_dsp, _y_dy_ddy_Foot_old, vec3(_Y_footStep,0,0));
        double yFoot = Params(0)*st_5 + Params(1)*st_4 + Params(2)*st_3 + Params(3)*st_2 + Params(4)*st_1 + Params(5);
        double dyFoot = 5*Params(0)*st_4 + 4*Params(1)*st_3 + 3*Params(2)*st_2 + 2*Params(3)*st_1 + Params(4);
        double ddyFoot = 20*Params(0)*st_3 + 12*Params(1)*st_2 + 6*Params(2)*st_1 + 2*Params(3);

        return vec3(yFoot, dyFoot, ddyFoot); //return y, dy, ddy
    }
    else
    {
    }
    return _y_dy_ddy_Foot_old;
}

vec3 Foot_yaw_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _yaw_d_dd_old, double _des_Yaw){
    double t_forward  = 0.9*_real_t_step*(1.0 - _dsp_ratio);
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;

    if(_t_foot_now < t_half_dsp - 0.5*dt)
    {
    }
    else if(_t_foot_now < t_forward + t_half_dsp - 0.5*dt){             //foot rising, 5th order trajectory up

        vec3 yaw_dp_ddp = calc_3rd(_t_foot_now - dt, t_forward + t_half_dsp, _yaw_d_dd_old, vec3(_des_Yaw,0,0));

        return yaw_dp_ddp;//vec3(xFoot, dxFoot, ddxFoot); //return z, dz, ddz
    }
    else
    {
    }

    return _yaw_d_dd_old;
}


double FootY_pos_limiter(char _swingFoot, vec3 _pPel, vec3 _pFoot_des, vec3 _pRF_current, vec3 _pLF_current){
    double safe_offset_y = 0.2;
    double reachable_dist = 0.6;
    vec3 pFoot_des = _pFoot_des;

    if(_swingFoot == 1){ // Left Foot
        if(pFoot_des.y < _pRF_current.y + safe_offset_y) pFoot_des.y = safe_offset_y + _pRF_current.y;
        if(pFoot_des.y > _pRF_current.y + reachable_dist) pFoot_des.y = _pRF_current.y + reachable_dist;

        return pFoot_des.y;
    }
    else if(_swingFoot == -1){ // Right Foot
        if(pFoot_des.y > _pLF_current.y - safe_offset_y) pFoot_des.y = _pLF_current.y - safe_offset_y;
        if(pFoot_des.y < _pLF_current.y - reachable_dist) pFoot_des.y = _pRF_current.y - reachable_dist;

        return pFoot_des.y;
    }
    else
        return pFoot_des.y;
}

double FootX_pos_limiter(vec3 _pPel, vec3 _pFoot_des){
    double reachable_dist = 0.2;
    vec3 pFoot_des = _pFoot_des;

    if(pFoot_des.x >= _pPel.x + reachable_dist) pFoot_des.x = _pPel.x + reachable_dist;
    else if(pFoot_des.x <= _pPel.x - reachable_dist) pFoot_des.x = _pPel.x - reachable_dist;

    return pFoot_des.x;
}

double FootPitchAngle_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, double _del_pitch_old, double _del_x){
    double take_off_ratio = 0.30;
    double arial_ratio = 0.30;
    double landing_ratio = 0.40;
    int t_half_dsp = (int)(_real_t_step*_dsp_ratio/2.0*freq);
    int t_take_off  = (int)(_real_t_step*take_off_ratio*(1.0 - _dsp_ratio)*freq);
    int t_arial  = (int)(_real_t_step*arial_ratio*(1.0 - _dsp_ratio)*freq);
    int t_landing = (int)(_real_t_step*landing_ratio*(1.0 - _dsp_ratio)*freq);
    int _t_foot_now_int = (int)(_t_foot_now*freq);

    double del_pitch;

    if(_t_foot_now_int <= t_half_dsp)
    {
        double del_pitch_angleFinal = 0.0;

        del_pitch = (del_pitch_angleFinal - _del_pitch_old)/(double)(t_half_dsp - _t_foot_now_int);

        return del_pitch;
    }
    else if(_t_foot_now_int <= t_take_off + t_half_dsp){             //foot rising
        del_pitch = _del_x/0.141;
        return del_pitch;
    }
    else if(_t_foot_now_int <= t_take_off + t_arial + t_half_dsp){
        if(_del_x > 0.00001){
            double del_pitch_angleFinal = 25.0;
            del_pitch = (del_pitch_angleFinal - _del_pitch_old)/(double)(t_take_off + t_arial + t_half_dsp - _t_foot_now_int);

            return del_pitch;
        }
        else if(_del_x < -0.00001){
            double del_pitch_angleFinal = -25.0;
            del_pitch = (del_pitch_angleFinal - _del_pitch_old)/(double)(t_take_off + t_arial + t_half_dsp - _t_foot_now_int);

            return del_pitch;
        }
        else{
            return _del_pitch_old;
        }
    }
    else if(_t_foot_now_int <= t_take_off + t_arial + t_landing + t_half_dsp)
    {
    }
    else{
        double del_pitch_angleFinal = 0.0;

        del_pitch = (del_pitch_angleFinal - _del_pitch_old)/(double)((int)(_real_t_step*freq) - _t_foot_now_int);

        return del_pitch;
    }

    return _del_pitch_old;
}

double LandingController(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _F_Foot, vec3 _X_dX_ddX_Foot, double _del_z_old){
    // Kenji Hashimoto, "Terrain-Adaptive Control With Small Landing Impact Force for Biped Vehicle" Reffered.
    double k1 = 1000.0; // N/m
    double c1 = 3000.0; // Ns/m
    double k2 = 14000.0; // N/m
    double del_L = 0.01; // m

    double m = 300;
    double c = 70000;
    double k = 50000;

    double del_z; // Fooz Z direction modification value

    int t_up  = (int)(_real_t_step*foot_up_ratio*(1.0 - _dsp_ratio)*freq);
    int t_half_dsp = (int)(_real_t_step*_dsp_ratio/2.0*freq);
    int t_down = (int)(_real_t_step*freq) - t_up - 2*t_half_dsp;
    int _t_foot_now_int = (int)(_t_foot_now*freq);

    if(_t_foot_now_int <= t_half_dsp)
    {
    }
    else if(_t_foot_now_int <= t_up + t_half_dsp) //foot rising,trajectory up
    {
    }
    else if(_t_foot_now_int > t_up + t_half_dsp && _t_foot_now_int <= t_up + 2*t_half_dsp + t_down){                  // foot down, 5th order trajectory down
        //if(_F_Foot.z <= 50) return _del_z_old;

        //del_z = (_F_Foot.z + c1/dt*_del_z_old - k2*del_L)/(k1 + c1/dt);
        double U = m*_X_dX_ddX_Foot[2] + c*_X_dX_ddX_Foot[1] + k*_X_dX_ddX_Foot[0] - _F_Foot.z;
        del_z  = _del_z_old*exp(-k/c*dt) - U/k*(exp(-k/c*dt) - 1);

        return del_z;
    }
    else
    {
    }

    return _del_z_old;

}


VectorNd calc_5th(double t_0, double t_e,vec3 h_0,vec3 h_e)
{
    MatrixNd AAA = MatrixNd::Zero(6,6);
    AAA(0,0) = pow(t_0,5);
    AAA(0,1) = pow(t_0,4);
    AAA(0,2) = pow(t_0,3);
    AAA(0,3) = pow(t_0,2);
    AAA(0,4) = pow(t_0,1);
    AAA(0,5) = 1;

    AAA(1,0) = 5*pow(t_0,4);
    AAA(1,1) = 4*pow(t_0,3);
    AAA(1,2) = 3*pow(t_0,2);
    AAA(1,3) = 2*pow(t_0,1);
    AAA(1,4) = 1;
    AAA(1,5) = 0;

    AAA(2,0) = 20*pow(t_0,3);
    AAA(2,1) = 12*pow(t_0,2);
    AAA(2,2) = 6*pow(t_0,1);
    AAA(2,3) = 2*1;
    AAA(2,4) = 0;
    AAA(2,5) = 0;

    AAA(3,0) = pow(t_e,5);
    AAA(3,1) = pow(t_e,4);
    AAA(3,2) = pow(t_e,3);
    AAA(3,3) = pow(t_e,2);
    AAA(3,4) = pow(t_e,1);
    AAA(3,5) = 1;

    AAA(4,0) = 5*pow(t_e,4);
    AAA(4,1) = 4*pow(t_e,3);
    AAA(4,2) = 3*pow(t_e,2);
    AAA(4,3) = 2*pow(t_e,1);
    AAA(4,4) = 1;
    AAA(4,5) = 0;

    AAA(5,0) = 20*pow(t_e,3);
    AAA(5,1) = 12*pow(t_e,2);
    AAA(5,2) = 6*pow(t_e,1);
    AAA(5,3) = 2*1;
    AAA(5,4) = 0;
    AAA(5,5) = 0;

    VectorNd BBB(6);
    BBB[0] = h_0[0];//position
    BBB[1] = h_0[1];//velocity
    BBB[2] = h_0[2];//acc
    BBB[3] = h_e[0];//position
    BBB[4] = h_e[1];//velocity
    BBB[5] = h_e[2];//acc

    return (AAA.inverse())*BBB;
}

vec3 calc_5th_GG(double t_now, double t_e, vec3 p_init, vec3 p_end){
        //Engineered by GG in 20170727
        double A, B, C, D, E, F;

        vec3 p_dp_ddp = vec3();


        //for safety
        if (t_now <= 0)
        {
            p_dp_ddp = p_init;
            return p_dp_ddp;
        }

        if(t_now >= t_e)
        {
            p_dp_ddp = p_end;
            return p_dp_ddp;
        }


        double tf1, tf2, tf3, tf4, tf5, pi, dpi, ddpi, pf;

        tf1 = t_e - t_now;
        tf2 = tf1*tf1;
        tf3 = tf2*tf1;
        tf4 = tf3*tf1;
        tf5 = tf4*tf1;

        pi = p_init[0];
        dpi = p_init[1];
        ddpi = p_init[2];

        pf = p_end[0];

        A = -(36*pi*tf2*tf2 + 18*ddpi*tf2*tf2*tf2 + 4*ddpi*tf3*tf3 - 36*pf*tf2*tf2 + 6*ddpi*tf1*tf1*tf4 + 36*dpi*tf1*tf2*tf2 - 24*dpi*tf1*tf1*tf3 - 24*pi*tf1*tf3 - 3*ddpi*tf2*tf4 + 6*dpi*tf1*tf4 - 12*dpi*tf2*tf3 + 24*pf*tf1*tf3 - 24*ddpi*tf1*tf2*tf3)/(2*(18*tf5*tf2*tf2 - 60*tf2*tf3*tf4 + 40*tf3*tf3*tf3 - 12*tf1*tf5*tf3 + 15*tf1*tf4*tf4));
        B =  -(20*dpi*tf3*tf3 + 20*ddpi*tf1*tf3*tf3 - 30*ddpi*tf2*tf2*tf3 - 6*ddpi*tf1*tf1*tf5 + 30*dpi*tf1*tf1*tf4 + 30*pi*tf1*tf4 - 60*pi*tf2*tf3 + 3*ddpi*tf2*tf5 - 5*ddpi*tf3*tf4 - 6*dpi*tf1*tf5 - 30*pf*tf1*tf4 + 60*pf*tf2*tf3 + 15*ddpi*tf1*tf2*tf4 - 60*dpi*tf1*tf2*tf3)/(2*(18*tf5*tf2*tf2 - 60*tf2*tf3*tf4 + 40*tf3*tf3*tf3 - 12*tf1*tf5*tf3 + 15*tf1*tf4*tf4));
        C =  -(80*pi*tf3*tf3 + 5*ddpi*tf4*tf4 - 80*pf*tf3*tf3 + 40*ddpi*tf2*tf3*tf3 - 30*ddpi*tf2*tf2*tf4 + 80*dpi*tf1*tf3*tf3 - 60*pi*tf2*tf4 - 4*ddpi*tf3*tf5 + 12*dpi*tf2*tf5 - 20*dpi*tf3*tf4 + 60*pf*tf2*tf4 + 12*ddpi*tf1*tf2*tf5 - 20*ddpi*tf1*tf3*tf4 - 60*dpi*tf1*tf2*tf4)/(2*(18*tf5*tf2*tf2 - 60*tf2*tf3*tf4 + 40*tf3*tf3*tf3 - 12*tf1*tf5*tf3 + 15*tf1*tf4*tf4));
        D = ddpi/2;
        E = dpi;
        F = pi;

        double t1 = dt;
        double t2 = t1*t1;
        double t3 = t2*t1;
        double t4 = t3*t1;
        double t5 = t4*t1;

        p_dp_ddp[0] = A*t5 + B*t4 + C*t3 + D*t2 + E*t1 + F;
        p_dp_ddp[1] = 5.0*A*t4 + 4.0*B*t3 + 3.0*C*t2 + 2.0*D*t1 + E;
        p_dp_ddp[2] = 20.0*A*t3 + 12.0*B*t2 + 6.0*C*t1 + 2.0*D;

        return p_dp_ddp;
    }


vec3 calc_3rd(double t_now, double t_e, vec3 p_init, vec3 p_end){

    // calculate 3rd order polynomial coefficient fot t_now to t_e
    // t_now : p_init(pi, dpi ,ddpi)
    // t_e   : p_end(pf, 0, 0)
    // and return p, dp ,ddp for t_now + dt


    double A, B, C, D;

    vec3 p_dp_ddp = vec3();


    //for safety
    if (t_now <= 0){
        p_dp_ddp = p_init;
        return p_dp_ddp;
    }
    if(t_now >= t_e){
        p_dp_ddp = p_end;
        return p_dp_ddp;
    }


    double tf1, tf2, tf3, pi, dpi, pf;

    tf1 = t_e - t_now;
    tf2 = tf1*tf1;
    tf3 = tf2*tf1;

    pi = p_init[0];
    dpi = p_init[1];

    pf = p_end[0];

    A = (2*(pi - pf) + dpi*tf1)/tf3;
    B = (-3*(pi - pf) - 2*dpi*tf1)/tf2;
    C = dpi;
    D = pi;

    double t1 = dt;
    double t2 = t1*t1;
    double t3 = t2*t1;

    p_dp_ddp[0] = A*t3 + B*t2 + C*t1 + D;
    p_dp_ddp[1] = 3*A*t2 + 2*B*t1 + C;
    p_dp_ddp[2] = 6*A*t1 + 2*B;

    return p_dp_ddp;
}

void calc_5th_param(double t_now, double t_e, vec3 p_init, vec3 p_end, double &A, double &B, double &C, double &D, double &E, double &F){
    vec3 p_dp_ddp = vec3();


    //for safety
    if (t_now <= 0){
        p_dp_ddp = p_init;
        A = B = C = D = E = F = 0;
    }
    if(t_now >= t_e){
        p_dp_ddp = p_end;
        A = B = C = D = E = F = 0;
    }


    double tf1, tf2, tf3, tf4, tf5, pi, dpi, ddpi, pf;

    tf1 = t_e - t_now;
    tf2 = tf1*tf1;
    tf3 = tf2*tf1;
    tf4 = tf3*tf1;
    tf5 = tf4*tf1;

    pi = p_init[0];
    dpi = p_init[1];
    ddpi = p_init[2];

    pf = p_end[0];

    A = -(36*pi*tf2*tf2 + 18*ddpi*tf2*tf2*tf2 + 4*ddpi*tf3*tf3 - 36*pf*tf2*tf2 + 6*ddpi*tf1*tf1*tf4 + 36*dpi*tf1*tf2*tf2 - 24*dpi*tf1*tf1*tf3 - 24*pi*tf1*tf3 - 3*ddpi*tf2*tf4 + 6*dpi*tf1*tf4 - 12*dpi*tf2*tf3 + 24*pf*tf1*tf3 - 24*ddpi*tf1*tf2*tf3)/(2*(18*tf5*tf2*tf2 - 60*tf2*tf3*tf4 + 40*tf3*tf3*tf3 - 12*tf1*tf5*tf3 + 15*tf1*tf4*tf4));
    B =  -(20*dpi*tf3*tf3 + 20*ddpi*tf1*tf3*tf3 - 30*ddpi*tf2*tf2*tf3 - 6*ddpi*tf1*tf1*tf5 + 30*dpi*tf1*tf1*tf4 + 30*pi*tf1*tf4 - 60*pi*tf2*tf3 + 3*ddpi*tf2*tf5 - 5*ddpi*tf3*tf4 - 6*dpi*tf1*tf5 - 30*pf*tf1*tf4 + 60*pf*tf2*tf3 + 15*ddpi*tf1*tf2*tf4 - 60*dpi*tf1*tf2*tf3)/(2*(18*tf5*tf2*tf2 - 60*tf2*tf3*tf4 + 40*tf3*tf3*tf3 - 12*tf1*tf5*tf3 + 15*tf1*tf4*tf4));
    C =  -(80*pi*tf3*tf3 + 5*ddpi*tf4*tf4 - 80*pf*tf3*tf3 + 40*ddpi*tf2*tf3*tf3 - 30*ddpi*tf2*tf2*tf4 + 80*dpi*tf1*tf3*tf3 - 60*pi*tf2*tf4 - 4*ddpi*tf3*tf5 + 12*dpi*tf2*tf5 - 20*dpi*tf3*tf4 + 60*pf*tf2*tf4 + 12*ddpi*tf1*tf2*tf5 - 20*ddpi*tf1*tf3*tf4 - 60*dpi*tf1*tf2*tf4)/(2*(18*tf5*tf2*tf2 - 60*tf2*tf3*tf4 + 40*tf3*tf3*tf3 - 12*tf1*tf5*tf3 + 15*tf1*tf4*tf4));
    D = ddpi/2;
    E = dpi;
    F = pi;
}

void calc_3rd_param(double t_now, double t_e, vec3 p_init, vec3 p_end, double &A, double &B, double &C, double &D){

    // calculate 3rd order polynomial coefficient fot t_now to t_e
    // t_now : p_init(pi, dpi ,ddpi)
    // t_e   : p_end(pf, 0, 0)
    // and return parameters : A, B, C, D

    vec3 p_dp_ddp = vec3();


    //for safety
    if (t_now <= 0){
        p_dp_ddp = p_init;
        A = B = C = D = 0;
    }
    if(t_now >= t_e){
        p_dp_ddp = p_end;
        A = B = C = D = 0;
    }


    double tf1, tf2, tf3, pi, dpi, pf;

    tf1 = t_e - t_now;
    tf2 = tf1*tf1;
    tf3 = tf2*tf1;

    pi = p_init[0];
    dpi = p_init[1];

    pf = p_end[0];

    A = (2*(pi - pf) + dpi*tf1)/tf3;
    B = (-3*(pi - pf) - 2*dpi*tf1)/tf2;
    C = dpi;
    D = pi;
}

void calc_1st_param(double t_now, double t_e, double p_init, double p_end, double &A, double &B){

    // calculate 1st order polynomial coefficient fot t_now to t_e
    // t_now : p_init
    // t_e   : p_end
    // and return parameters : A, B


    //for safety
    if (t_now <= 0){
        A = B = 0;
    }
    if(t_now >= t_e){
        A = B = 0;
    }


    double tf1, pi, pf;

    tf1 = t_e - t_now;


    pi = p_init;

    pf = p_end;

    A = (pf - pi)/tf1;
    B = pi;
}

double calc_trap(double _del_t, double _te, double _he, double _t_now){
    //       /-------\  - - - -  h_e
    //      /_________\ _ _ _ _  0
    //     0 del_t     te
    //     t2 = te - del_t

    //vec3 out_ = vec3();
    double out_= 0;
    double a1, a2;
    if(_t_now < _del_t - 0.5*dt){
        a1 = _he/_del_t;
        out_ = a1*_t_now;
        //out_[0] = 0.5*a1*_t_now*_t_now; // area of Trapezoidal profile (distance)

        return out_;
    }
    else if(_t_now < _te - _del_t - 0.5*dt){
        out_ = _he;
        //out_[0] = 0.5*a1*_del_t*_del_t + _he*(_t_now - _del_t);
        //cout<<"out: "<<out_<<endl;
        return out_;
    }
    else{
        a2 = -_he/(_del_t);
        double b = -a2*_te;

        out_ = a2*_t_now + b;
        //out_[0] = 0.5*a1*_del_t*_del_t + _he*(_te - _del_t) + 0.5*a2*(_t_now - (_te - _del_t))*(_t_now - (_te - _del_t)) + b*(_t_now - (_te - _del_t));
        return out_;
    }

}


VectorNd AnkleFT_Calculator(vec3 _ZMP_ref, quat _qPel, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF){

    if(_F_RF.z <= 0) _F_RF.z = 0;
    if(_F_LF.z <= 0) _F_LF.z = 0;
    // Generate reference torque according to control mode : position control, zmp control
    double Alpha = 0.50;
    VectorNd Target_Joint_Torque_Fz = VectorNd::Zero(6); // {R_Tx, R_Ty, L_Tx, L_Ty, R_Fz, L_Fz}

    // Calculate alpha : distributino ratio in Kajita's paper ==============================
    vec3 LF_lt1 = vec3(FootLength/2, FootWidth/2, 0);
    vec3 LF_rt2 = vec3(FootLength/2, -FootWidth/2, 0);
    vec3 LF_rb3 = vec3(-FootLength/2, -FootWidth/2, 0);
    vec3 LF_lb4 = vec3(-FootLength/2, FootWidth/2, 0);

    vec3 RF_lt1 = vec3(FootLength/2, FootWidth/2, 0);
    vec3 RF_rt2 = vec3(FootLength/2, -FootWidth/2, 0);
    vec3 RF_rb3 = vec3(-FootLength/2, -FootWidth/2, 0);
    vec3 RF_lb4 = vec3(-FootLength/2, FootWidth/2, 0);

    LF_lt1 = LF_lt1*mat3(_qLF);
    LF_rt2 = LF_rt2*mat3(_qLF);
    LF_rb3 = LF_rb3*mat3(_qLF);
    LF_lb4 = LF_lb4*mat3(_qLF);

    RF_lt1 = RF_lt1*mat3(_qRF);
    RF_rt2 = RF_rt2*mat3(_qRF);
    RF_rb3 = RF_rb3*mat3(_qRF);
    RF_lb4 = RF_lb4*mat3(_qRF);

    LF_lt1 = LF_lt1 + _pLF;
    LF_rt2 = LF_rt2 + _pLF;
    LF_rb3 = LF_rb3 + _pLF;
    LF_lb4 = LF_lb4 + _pLF;

    RF_lt1 = RF_lt1 + _pRF;
    RF_rt2 = RF_rt2 + _pRF;
    RF_rb3 = RF_rb3 + _pRF;
    RF_lb4 = RF_lb4 + _pRF;

    // Calc Alpha (distribution ratio)
    if(_F_RF.z > 100 && _F_LF.z <= 100){ // RF SSP phase
        Alpha = 1;
    }
    else if(_F_RF.z <= 100 && _F_LF.z > 100){ // LF SSP phase
        Alpha = 0;
    }
    else{ // DSP phase
        // Check wheather _ZMP_ref is in LF SP or RF SP
        stack<Point> SP_LF;
        Point points_LF[4];
        points_LF[0] = {LF_lt1.x, LF_lt1.y};
        points_LF[1] = {LF_rt2.x, LF_rt2.y};
        points_LF[2] = {LF_rb3.x, LF_rb3.y};
        points_LF[3] = {LF_lb4.x, LF_lb4.y};
        SP_LF.push({LF_lt1.x, LF_lt1.y});
        SP_LF.push({LF_rt2.x, LF_rt2.y});
        SP_LF.push({LF_rb3.x, LF_rb3.y});
        SP_LF.push({LF_lb4.x, LF_lb4.y});

        stack<Point> SP_RF;
        Point points_RF[4];
        points_RF[0] = {RF_lt1.x, RF_lt1.y};
        points_RF[1] = {RF_rt2.x, RF_rt2.y};
        points_RF[2] = {RF_rb3.x, RF_rb3.y};
        points_RF[3] = {RF_lb4.x, RF_lb4.y};
        SP_RF.push({RF_lt1.x, RF_lt1.y});
        SP_RF.push({RF_rt2.x, RF_rt2.y});
        SP_RF.push({RF_rb3.x, RF_rb3.y});
        SP_RF.push({RF_lb4.x, RF_lb4.y});

        Point ZMP_ref = {_ZMP_ref.x, _ZMP_ref.y};

        if(zmpInOutCheck(ZMP_ref, SP_LF) == true) Alpha = 0;        // ZMP_ref is in LF
        else if(zmpInOutCheck(ZMP_ref, SP_RF) == true) Alpha = 1;   // ZMP_ref is in RF
        else // ZMP_ref is in middle of two foot
        {    // Calculate most closest point in each foot P_RF, P_LF from _ZMP_ref

//            // get P_RF ---------------------------------------------------------------------------------------------------------------
//            Point SPs[4];
//            Point zmp = {_ZMP_ref.x, _ZMP_ref.y};

//            double centerX = 0,centerY = 0;

//            for(int i=0; i<4 ;i++){
//                SPs[i] = points_RF[i];
//                centerX += SPs[i].x;
//                centerY += SPs[i].y;
//            }
//            Point center = {centerX/4,centerY/4};
//            int first_dist_index=0, second_dist_index=1;
//            for(int i=0; i<4 ; i++){
//                int j = i+1;
//                if(j == 4) j = 0;
//                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) < 0.00001 &&
//                            ((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) > -0.00001){
//                    i++;
//                    int j = i+1;
//                    if(j == 4) j = 0;
//                }
//                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
//                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

//                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

//                int numOfsp=100;

//                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

//                if(numOfsp == 4){
//                    if(zmpInOutCheck(cross_point,littleSP) == true){
//                        cout<<"i : "<<i<<"numOfsp: "<<numOfsp<<endl;
//                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
//                            first_dist_index = i;
//                            second_dist_index = j;
//                        }
//                        else{
//                            first_dist_index = j;
//                            second_dist_index = i;
//                        }
//                    }
//                }
//            }

//            Point sp1 = SPs[first_dist_index];
//            Point sp2 = SPs[second_dist_index];

////            cout<<"first index: "<<first_dist_index<<"  x : "<<sp1.x<<" y: "<<sp1.y<<endl;
////            cout<<"second index: "<<second_dist_index<<"  x : "<<sp2.x<<" y: "<<sp2.y<<endl;

//            double c_sqare = distSq(zmp, sp2); // long side of triangle
//            double a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
//            double b_sqare = distSq(zmp, sp1); // short side of triangle

//            vec3 P_RF;

//            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
//                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
//                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

//                vec3 p2n_proj = n*(dot(p,n)/dot(n));

//                P_RF = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
//            }
//            else{
//                P_RF = vec3(sp1.x, sp1.y, 0);
//            }
//            //-----------------------------------------------------------------------------------------------------

//            // get P_LF-------------------------------------------------------------------------------------------
//            for(int i=0; i<4 ;i++){
//                SPs[i] = points_LF[i];
//                centerX += SPs[i].x;
//                centerY += SPs[i].y;
//            }
//            center = {centerX/4,centerY/4};

//            for(int i=0; i<4 ; i++){
//                int j = i+1;
//                if(j == 4) j = 0;
//                if(((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) < 0.00001 &&
//                            ((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)) > -0.00001){
//                    i++;
//                    int j = i+1;
//                    if(j == 4) j = 0;
//                }
//                Point cross_point = {((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.x - zmp.x) - (SPs[i].x - SPs[j].x)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x)),
//                                     ((SPs[i].x*SPs[j].y - SPs[i].y*SPs[j].x)*(center.y - zmp.y) - (SPs[i].y - SPs[j].y)*(center.x*zmp.y - center.y*zmp.x))/((SPs[i].x - SPs[j].x)*(center.y - zmp.y)-(SPs[i].y - SPs[j].y)*(center.x - zmp.x))};

//                Point littlePoints[] = {SPs[i],SPs[j],center,zmp};

//                int numOfsp=100;

//                stack<Point> littleSP = convexHull(littlePoints,4,numOfsp);

//                if(numOfsp == 4){
//                    if(zmpInOutCheck(cross_point,littleSP) == true){
//                        if(distSq(zmp,SPs[i]) <= distSq(zmp,SPs[j])){
//                            first_dist_index = i;
//                            second_dist_index = j;
//                        }
//                        else{
//                            first_dist_index = j;
//                            second_dist_index = i;
//                        }
//                    }
//                }
//            }

//            sp1 = SPs[first_dist_index];
//            sp2 = SPs[second_dist_index];

//            c_sqare = distSq(zmp, sp2); // long side of triangle
//            a_sqare = distSq(sp1, sp2); // dist between two point of Supporting polygon
//            b_sqare = distSq(zmp, sp1); // short side of triangle

//            vec3 P_LF;

//            if(acos((a_sqare + b_sqare - c_sqare)/(2*sqrt(a_sqare)*sqrt(b_sqare))) < PI/2.0){
//                vec3 n = vec3(sp2.x - sp1.x, sp2.y - sp1.y, 0);
//                vec3 p = vec3(zmp.x - sp1.x, zmp.y - sp1.y, 0);

//                vec3 p2n_proj = n*(dot(p,n)/dot(n));

//                P_LF = vec3(sp1.x + p2n_proj.x, sp1.y + p2n_proj.y,0);
//            }
//            else{
//                P_LF = vec3(sp1.x, sp1.y, 0);
//            }
//            //-----------------------------------------------------------------------------------------------------------------

            vec3 P_LF = _pLF;
            vec3 P_RF = _pRF;
            vec3 zmp = _ZMP_ref;

            // get P_alpha : closest point on line P_LF & P_RF from _ZMP_ref
            vec3 n = vec3(P_LF.x - P_RF.x, P_LF.y - P_RF.y, 0);
            vec3 p = vec3(zmp.x - P_RF.x, zmp.y - P_RF.y, 0);

            vec3 p2n_proj = n*(dot(p,n)/dot(n));

            vec3 P_alpha = vec3(P_RF.x + p2n_proj.x, P_RF.y + p2n_proj.y,0);

            // get alpha
            Alpha = sqrt(distSq({P_LF.x, P_LF.y},{P_alpha.x, P_alpha.y}))/sqrt(distSq({P_LF.x, P_LF.y},{P_RF.x, P_RF.y}));
            //cout<<"P_LF: ("<<P_LF.x<<", "<<P_LF.y<<") P_RF: ("<<P_RF.x<<", "<<P_RF.y<<")  alpha : "<<Alpha<<endl;
            //cout<<"Alpha base : "<<sqrt(distSq({P_LF.x, P_LF.y},{P_RF.x, P_RF.y}))<<endl;
            //cout<<"P_LF : "<<P_LF.x<<", "<<P_LF.y<<"   P_RF : "<<P_RF.x<<", "<<P_RF.y<<endl;  
        }
        //cout<<"alpha : "<<Alpha<<endl;
    }

    // Calculate Alpha Done===========================================================================

    // Calculate Alpha 2
    vec3 P_LF = _pLF;
    vec3 P_RF = _pRF;
    vec3 zmp = _ZMP_ref;

    // get P_alpha : closest point on line P_LF & P_RF from _ZMP_ref
    vec3 n = vec3(P_LF.x - P_RF.x, P_LF.y - P_RF.y, 0);
    vec3 p = vec3(zmp.x - P_RF.x, zmp.y - P_RF.y, 0);

    vec3 p2n_proj = n*(dot(p,n)/dot(n));

    vec3 P_alpha = vec3(P_RF.x + p2n_proj.x, P_RF.y + p2n_proj.y,0);

    // get alpha
    double Alpha2 = sqrt(distSq({P_LF.x, P_LF.y},{P_alpha.x, P_alpha.y}))/sqrt(distSq({P_LF.x, P_LF.y},{P_RF.x, P_RF.y}));
    if(Alpha2 >= 1.0) Alpha2 = 1.0;
    //cout<<"Alpha2 : "<<Alpha2<<endl;


    // Calculate Sum of reference Torque Tr + Tl
    vec3 SumOfTorque = -cross(mat3(_qPel)*(_pRF - _ZMP_ref), mat3(_qPel)*_F_RF) - cross(mat3(_qPel)*(_pLF - _ZMP_ref), mat3(_qPel)*_F_LF);

    vec3 LF2RF = _pRF - _pLF;
    vec3 LF2RF_wrtPel = mat3(_qPel)*LF2RF;
    double TF_angle = atan(LF2RF_wrtPel.x/(-LF2RF_wrtPel.y)); // return -pi/2 ~ pi/2  in radian

    vec3 SumOfTorque_local = vec3(0,0,0);
    SumOfTorque_local.x = SumOfTorque.x*cos(TF_angle) + SumOfTorque.y*sin(TF_angle);
    SumOfTorque_local.y = -SumOfTorque.x*sin(TF_angle) + SumOfTorque.y*cos(TF_angle);

    // Distribute SumOfTorque_local (local means ... T' in Kajita's paper)
    vec3 TorqueRF, TorqueLF, TorqueRF_local, TorqueLF_local;
    TorqueRF_local.y = Alpha*SumOfTorque_local.y;
    TorqueLF_local.y = (1.0 - Alpha)*SumOfTorque_local.y;

//        if(SumOfTorque_local.x < 0){
//            TorqueRF_local.x = SumOfTorque_local.x;
//            TorqueLF_local.x = 0;
//        }
//        else{
//            TorqueRF_local.x = 0;
//            TorqueLF_local.x = SumOfTorque_local.x;
//        }
    // i dont agree with kajita's rule of ditribute torque x

    TorqueRF_local.x = Alpha*SumOfTorque_local.x;
    TorqueLF_local.x = (1.0 - Alpha)*SumOfTorque_local.x;

    // Calculate torque of each foot joint  // Need some modification in case of qRF,qLF are not parellel to qPel
    TorqueRF.x = TorqueRF_local.x*cos(-TF_angle) + TorqueRF_local.y*sin(-TF_angle);
    TorqueRF.y = -TorqueRF_local.x*sin(-TF_angle) + TorqueRF_local.y*cos(-TF_angle);

    TorqueLF.x = TorqueLF_local.x*cos(-TF_angle) + TorqueLF_local.y*sin(-TF_angle);
    TorqueLF.y = -TorqueLF_local.x*sin(-TF_angle) + TorqueLF_local.y*cos(-TF_angle);

    // Calculate target Fz of each foot
    Target_Joint_Torque_Fz[0] = - TorqueRF.x;                     // RF Tx
    Target_Joint_Torque_Fz[1] = - TorqueRF.y;                     // RF Ty
    Target_Joint_Torque_Fz[2] = - TorqueLF.x;                     // LF Tx
    Target_Joint_Torque_Fz[3] = - TorqueLF.y;                     // LF Ty
    Target_Joint_Torque_Fz[4] = Alpha2*(_F_RF.z + _F_LF.z);       // RF Fz
    Target_Joint_Torque_Fz[5] = (1.0 - Alpha2)*(_F_RF.z + _F_LF.z); // LF Fz

    return Target_Joint_Torque_Fz;

}

vec4 zmpFeedBackController(int _on_off, vec3 _ZMP_ref, vec3 _dZMP_ref, vec3 _ZMP_m, vec3 _dZMP_m, quat _qRF, quat _qLF,
                              vec3 _F_RF, vec3 _F_LF){

    vec4 Feedback_Joint_Torque;

    if(_on_off == 0){
        Feedback_Joint_Torque[0] = 0;
        Feedback_Joint_Torque[1] = 0;
        Feedback_Joint_Torque[2] = 0;
        Feedback_Joint_Torque[3] = 0;
        return Feedback_Joint_Torque;
    }

    if(_F_RF.z <= 0) _F_RF.z = 0;
    if(_F_LF.z <= 0) _F_LF.z = 0;


    double Pgain = 0;//1;//10;
    double Dgain = 0;//0.1;//0.3;
    vec3 ZMP_errorVec = _ZMP_ref - _ZMP_m;
    vec3 dZMP_errorVec = _dZMP_ref - _dZMP_m;

    if(_F_RF.z <= 20){ // LF SSP phase
        vec3 ZMP_errorVec_wrtLF = mat3(_qLF)*ZMP_errorVec;
        vec3 dZMP_errorVec_wrtLF = mat3(_qLF)*dZMP_errorVec;

        Feedback_Joint_Torque[0] = 0;
        Feedback_Joint_Torque[1] = 0;
        Feedback_Joint_Torque[2] = -(Pgain*ZMP_errorVec_wrtLF.y + Dgain*dZMP_errorVec_wrtLF.y);
        Feedback_Joint_Torque[3] = Pgain*ZMP_errorVec_wrtLF.x + Dgain*dZMP_errorVec_wrtLF.x;
    }
    if(_F_LF.z <= 20){ // RF SSP phase
        vec3 ZMP_errorVec_wrtRF = mat3(_qRF)*ZMP_errorVec;
        vec3 dZMP_errorVec_wrtRF = mat3(_qRF)*dZMP_errorVec;

        Feedback_Joint_Torque[0] = -(Pgain*ZMP_errorVec_wrtRF.y + Dgain*dZMP_errorVec_wrtRF.y);
        Feedback_Joint_Torque[1] = Pgain*ZMP_errorVec_wrtRF.x + Dgain*dZMP_errorVec_wrtRF.x;
        Feedback_Joint_Torque[2] = 0;
        Feedback_Joint_Torque[3] = 0;
    }

    return Feedback_Joint_Torque;
}

vec4 AnkleJointAngleContoller(int _ctrl_mode, vec3 _F_RF, vec3 _F_LF, vec4 _R_Ank_angVel, vec4 _L_Ank_angVel,
                                vec4 _R_Ank_ref, vec4 _L_Ank_ref){
    double RF_roll_ref = _R_Ank_ref[0];
    double RF_roll_vel_ref = _R_Ank_ref[1];
    double RF_pitch_ref = _R_Ank_ref[2];
    double RF_pitch_vel_ref = _R_Ank_ref[3];
    double LF_roll_ref = _L_Ank_ref[0];
    double LF_roll_vel_ref = _L_Ank_ref[1];
    double LF_pitch_ref = _L_Ank_ref[2];
    double LF_pitch_vel_ref = _L_Ank_ref[3];

    double RF_roll = _R_Ank_angVel[0];
    double RF_roll_vel = _R_Ank_angVel[1];
    double RF_pitch = _R_Ank_angVel[2];
    double RF_pitch_vel = _R_Ank_angVel[3];
    double LF_roll = _L_Ank_angVel[0];
    double LF_roll_vel = _L_Ank_angVel[1];
    double LF_pitch = _L_Ank_angVel[2];
    double LF_pitch_vel = _L_Ank_angVel[3];

    vec4 Feedback_Joint_Torque;

    if(_ctrl_mode == 0){
        Feedback_Joint_Torque[0] = 0;
        Feedback_Joint_Torque[1] = 0;
        Feedback_Joint_Torque[2] = 0;
        Feedback_Joint_Torque[3] = 0;
        return Feedback_Joint_Torque;
    }

    else if(_ctrl_mode == 1){  // Position Control Mode
        double Pgain_pos = 50.0;
        double Dgain_pos = 0.2;
        Feedback_Joint_Torque[0] = Pgain_pos*(RF_roll_ref - RF_roll) + Dgain_pos*(RF_roll_vel_ref - RF_roll_vel);
        Feedback_Joint_Torque[1] = 3/2*Pgain_pos*(RF_pitch_ref - RF_pitch) + Dgain_pos*(RF_pitch_vel_ref - RF_pitch_vel);
        Feedback_Joint_Torque[2] = Pgain_pos*(LF_roll_ref - LF_roll) + Dgain_pos*(LF_roll_vel_ref - LF_roll_vel);
        Feedback_Joint_Torque[3] = 3/2*Pgain_pos*(LF_pitch_ref - LF_pitch) + Dgain_pos*(LF_pitch_vel_ref - LF_pitch_vel);

        return Feedback_Joint_Torque;
    }

    else if(_ctrl_mode == 2){ // arial phase position control
        double low_R_Pgain_roll;
        double low_R_Dgain_roll;
        double low_R_Pgain_pitch;
        double low_R_Dgain_pitch;

        double low_L_Pgain_roll;
        double low_L_Dgain_roll;
        double low_L_Pgain_pitch;
        double low_L_Dgain_pitch;

        double FF_R_gain, FF_L_gain;

        if(_F_RF.z <= 50){ // LF SSP phase
            // Right ankle need position control to maintain its orientation in aireal phase
            low_R_Pgain_roll = 1.5;//10;
            low_R_Dgain_roll = 0.01;  // low gain in arial phase
            low_R_Pgain_pitch = 1;
            low_R_Dgain_pitch = 0.05;
            FF_R_gain = 0.3;

        }
        else{
            low_R_Pgain_roll = 0.0;
            low_R_Dgain_roll = 0.25;
            low_R_Pgain_pitch =0.0;
            low_R_Dgain_pitch =0.1;
            FF_R_gain = 0.0;
        }
        double FF_RAR = RF_roll_vel_ref*FF_R_gain;
        double FF_RAP = RF_pitch_vel_ref*FF_R_gain;

        Feedback_Joint_Torque[0] = FF_RAR + low_R_Pgain_roll*(RF_roll_ref - RF_roll) + low_R_Dgain_roll*(RF_roll_vel_ref - RF_roll_vel);
        Feedback_Joint_Torque[1] = FF_RAP + low_R_Pgain_pitch*(RF_pitch_ref - RF_pitch) + low_R_Dgain_pitch*(RF_pitch_vel_ref - RF_pitch_vel);

        if(_F_LF.z <= 50){ // RF SSP phase
            // Left ankle need position control to maintain its orientation in aireal phase
            low_L_Pgain_roll = 1.5;
            low_L_Dgain_roll = 0.01;  // low gain in arial phase
            low_L_Pgain_pitch = 1;
            low_L_Dgain_pitch = 0.05;  // low gain in arial phase
            FF_L_gain = 0.3;
        }
        else{
            low_L_Pgain_roll = 0.0;
            low_L_Dgain_roll = 0.25;
            low_L_Pgain_pitch =0.0;
            low_L_Dgain_pitch =0.1;
            FF_L_gain = 0.0;
        }
        double FF_LAR = LF_roll_vel_ref*FF_L_gain;
        double FF_LAP = LF_pitch_vel_ref*FF_L_gain;

        Feedback_Joint_Torque[2] = FF_LAR + low_L_Pgain_roll*(LF_roll_ref - LF_roll) + low_L_Dgain_roll*(LF_roll_vel_ref - LF_roll_vel);
        Feedback_Joint_Torque[3] = FF_LAP + low_L_Pgain_pitch*(LF_pitch_ref - LF_pitch) + low_L_Dgain_pitch*(LF_pitch_vel_ref - LF_pitch_vel);
    }
    return Feedback_Joint_Torque;

}


vec3 ComDampingController(vec3 _COM_con_old, vec3 _COM_input, vec3 _dCOM_input, vec3 _COM_m, vec3 _dCOM_m, vec3 _F_RF, vec3 _F_LF ){
    vec3 COM_con_final = vec3(0,0,0);
    vec3 COM_con = vec3(0,0,0);
    vec3 COM_differ = _COM_m - _COM_input;
    vec3 dCOM_differ = _dCOM_m - _dCOM_input;

    if(_F_RF.z > 50 && _F_LF.z > 50){
//        COM_con.y = -(-0.4627*COM_differ.y + 0.021*dCOM_differ.y);
        COM_con.y = -0.2*(-0.4627*COM_differ.y + 0.0211*dCOM_differ.y);
        if(COM_con.y > 0.05) COM_con.y = 0.05;
        if(COM_con.y < -0.05) COM_con.y = -0.05;
    }
    else{
        COM_con = vec3(0,0,0);//_COM_con_old;
    }

    double alpha = 0.8;
    COM_con_final = (1 - alpha)*_COM_con_old + alpha*COM_con;


    return COM_con_final;
}

//Ankle Torque Control---------------------------------------------------------------------

void AnkleToruqeControl_Init(void){
    // RAR
    MCsetFrictionParameter(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 120, 50, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, DISABLE);
    MCJointGainOverride(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 100, 800);

    // RAP
    MCsetFrictionParameter(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 120, 50, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, DISABLE);
    MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 100, 800);

    // LAR
    MCsetFrictionParameter(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 120, 50, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, DISABLE);
    MCJointGainOverride(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 100, 800);

    // LAP
    MCsetFrictionParameter(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 120, 50, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, DISABLE);
    MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 100, 800);

}

void AnkleTorqueControl_Stop(void){
    MCenableFrictionCompensation(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RAR].canch,JOINT_INFO[RAR].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LAR].canch,JOINT_INFO[LAR].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch,JOINT_INFO[LAP].bno, SW_MODE_COMPLEMENTARY);


    MCJointPWMCommand2chHR(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, 4, 0, 0, 0);


    MCJointGainOverride(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 0,2000); //--RAR
    MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 0,2000); //--RAP
    MCJointGainOverride(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 0,2000); //--LAR
    MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 0,2000); //--LAP

    usleep(500*1000);

    MCJointPWMCommand2chHR(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, 4, 0, 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, 4, 0, 0, 0);
}

void AnkleTorqueControl(double _RAR_T_ref, double _RAP_T_ref, double _LAR_T_ref, double _LAP_T_ref,
                        vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF){

    // Ampere input
    int RAR_input = 0;
    int RAP_input = 0;
    int LAR_input = 0;
    int LAP_input = 0;

    double Pgain_R_roll;
    double Dgain_R_roll;
    double Igain_R_roll;

    double Pgain_R_pitch;
    double Dgain_R_pitch;
    double Igain_R_pitch;

    double Pgain_L_roll;
    double Dgain_L_roll;
    double Igain_L_roll;

    double Pgain_L_pitch;
    double Dgain_L_pitch;
    double Igain_L_pitch;

    static double _E_RAR_old = 0;
    static double _E_RAP_old = 0;
    static double _E_LAR_old = 0;
    static double _E_LAP_old = 0;

    static double _E_RAR_sum = 0;
    static double _E_RAP_sum = 0;
    static double _E_LAR_sum = 0;
    static double _E_LAP_sum = 0;

    static double RAR_C_FB = 0;
    static double RAP_C_FB = 0;
    static double LAR_C_FB = 0;
    static double LAP_C_FB = 0;


    // Feedforward
    int RAR_C_FF = torque_OLcurrent_map_RAR(_RAR_T_ref);
    int RAP_C_FF = torque_OLcurrent_map_RAP(_RAP_T_ref);
    int LAR_C_FF = torque_OLcurrent_map_LAR(_LAR_T_ref);
    int LAP_C_FF = torque_OLcurrent_map_LAP(_LAP_T_ref);

    // FeedBack
    if(_F_RF.norm() > 50.0){
        Pgain_R_roll = 13;
        Dgain_R_roll = 0.3;
        Igain_R_roll = 0.2;

        Pgain_R_pitch = 20;
        Dgain_R_pitch = 0.1;//
        Igain_R_pitch = 0.4;
    }
    else{
        Pgain_R_roll = 0;
        Dgain_R_roll = 0;
        Igain_R_roll = 0;

        Pgain_R_pitch = 0;
        Dgain_R_pitch = 0;
        Igain_R_pitch = 0;

        _E_RAR_sum -= _E_RAR_sum/30.0;
        _E_RAP_sum -= _E_RAP_sum/30.0;
    }

    if(_F_LF.norm() > 50.0){
        Pgain_L_roll = 13;
        Dgain_L_roll = 0.3;
        Igain_L_roll = 0.2;

        Pgain_L_pitch = 20;
        Dgain_L_pitch = 0.1;//
        Igain_L_pitch = 0.4;
    }
    else{
        Pgain_L_roll = 0;
        Dgain_L_roll = 0;
        Igain_L_roll = 0;

        Pgain_L_pitch = 0;
        Dgain_L_pitch = 0;
        Igain_L_pitch = 0;

        _E_LAR_sum -= _E_LAR_sum/30.0;
        _E_LAP_sum -= _E_LAP_sum/30.0;
    }

    double _E_RAR_cur = _RAR_T_ref - _M_RF.x;
    double _E_RAP_cur = _RAP_T_ref - _M_RF.y;
    double _E_LAR_cur = _LAR_T_ref - _M_LF.x;
    double _E_LAP_cur = _LAP_T_ref - _M_LF.y;
    double dE_RAR_cur = (_E_RAR_cur - _E_RAR_old)/dt;
    double dE_RAP_cur = (_E_RAP_cur - _E_RAP_old)/dt;
    double dE_LAR_cur = (_E_LAR_cur - _E_LAR_old)/dt;
    double dE_LAP_cur = (_E_LAP_cur - _E_LAP_old)/dt;

    _E_RAR_sum += _E_RAR_cur;
    _E_RAP_sum += _E_RAP_cur;
    _E_LAR_sum += _E_LAR_cur;
    _E_LAP_sum += _E_LAP_cur;

    int RAR_C_FB_new = (int)(_E_RAR_cur*Pgain_R_roll + dE_RAR_cur*Dgain_R_roll + _E_RAR_sum*Igain_R_roll);
    int RAP_C_FB_new = (int)(_E_RAP_cur*Pgain_R_pitch + dE_RAP_cur*Dgain_R_pitch + _E_RAP_sum*Igain_R_pitch);
    int LAR_C_FB_new = (int)(_E_LAR_cur*Pgain_L_roll + dE_LAR_cur*Dgain_L_roll + _E_LAR_sum*Igain_L_roll);
    int LAP_C_FB_new = (int)(_E_LAP_cur*Pgain_L_pitch + dE_LAP_cur*Dgain_L_pitch + _E_LAP_sum*Igain_L_pitch);

    double alpha_FB_pitch = 1/(1 + 2*PI*dt*4);
    double alpha_FB_roll = 1/(1 + 2*PI*dt*4);
    RAR_C_FB = RAR_C_FB*alpha_FB_roll + RAR_C_FB_new*(1.0-alpha_FB_roll);
    RAP_C_FB = RAP_C_FB*alpha_FB_pitch + RAP_C_FB_new*(1.0-alpha_FB_pitch);
    LAR_C_FB = LAR_C_FB*alpha_FB_roll + LAR_C_FB_new*(1.0-alpha_FB_roll);
    LAP_C_FB = LAP_C_FB*alpha_FB_pitch + LAP_C_FB_new*(1.0-alpha_FB_pitch);

    RAR_input = RAR_C_FF + RAR_C_FB;
    RAP_input = RAP_C_FF + RAP_C_FB;
    LAR_input = LAR_C_FF + LAR_C_FB;
    LAP_input = LAP_C_FF + LAP_C_FB;

    if(RAR_input >= 300) RAR_input = 300;
    if(RAR_input <= -300) RAR_input = -300;
    if(RAP_input >= 800) RAP_input = 800;
    if(RAP_input <= -800) RAP_input = -800;

    if(LAR_input >= 300) LAR_input = 300;
    if(LAR_input <= -300) LAR_input = -300;
    if(LAP_input >= 800) LAP_input = 800;
    if(LAP_input <= -800) LAP_input = -800;

    MCJointPWMCommand2chHR(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, 4, RAR_input, 0, 0); //RAR
    MCJointPWMCommand2chHR(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, 4, RAP_input, 0, 0); //RAP

    MCJointPWMCommand2chHR(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, 4, LAR_input, 0, 0); //LAR
    MCJointPWMCommand2chHR(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, 4, LAP_input, 0, 0); //LAP

//    cout<<"cntrol  ref: "<<_RAR_T_ref<<"  measure : "<<_RAR_T_mea<<endl;
//    cout<<"LAR_C_FF: "<<LAR_C_FF<<"  LAR_C_FB : "<<LAR_C_FB<<" LAR_input : "<<LAR_input<<endl;

    _E_RAR_old = _E_RAR_cur;
    _E_RAP_old = _E_RAP_cur;
    _E_LAR_old = _E_LAR_cur;
    _E_LAP_old = _E_LAP_cur;
}



//---------------Torque current Mapping----------------------------------------------------------------
#define SIGN(x) (x>=0 ? 1:-1)
int torque_OLcurrent_map_RAR(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = 1;
    const int n = 6;
    const double t_table[6] = {0, 1.5, 7.2, 12.6, 22.0, 36.0};
    const double c_table[6] = {0, 35,   55, 85, 200, 300};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_RAP(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = 1;
    const int n = 6;
    const double t_table[6] = {0, 3.5, 7.0, 11.5, 26.0, 42.0};
    const double c_table[6] = {0, 55,   95, 145, 500, 700};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LAR(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = 1;
    const int n = 6;
    const double t_table[6] = {0, 1.5, 7.2, 12.6, 22.0, 36.0};
    const double c_table[6] = {0, 35,   55, 85, 200, 300};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LAP(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = 1;
    const int n = 6;
    const double t_table[6] = {0, 3.5, 7.0, 11.5, 26.0, 42.0};
    const double c_table[6] = {0, 55,   95, 145, 500, 700};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}


vec3 HipRoll_Compensation(char _swingFoot, double _real_t_step, double _t_foot_now, double _dsp_ratio){
    double t_up  = _real_t_step*foot_up_ratio*(1.0 - _dsp_ratio);
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;
    double t_down = _real_t_step - t_up - 2*t_half_dsp;

    double angleMax = 2.0;
    double RHR_comp_ang = 0;
    double LHR_comp_ang = 0;

    if(_t_foot_now <  t_half_dsp - 0.5*dt)
    {
        RHR_comp_ang = 0;
        LHR_comp_ang = 0;
    }
    else if(_t_foot_now < t_up + t_half_dsp - 0.5*dt){             //foot rising, 5th order trajectory up

        double compAngle = (angleMax/t_up)*(_t_foot_now - t_half_dsp);

        if(_swingFoot == 1){ // Left Foot
            RHR_comp_ang = compAngle*(-1);
        }
        else if(_swingFoot == -1){ // Right Foot
            LHR_comp_ang = compAngle;
        }
        else{
            RHR_comp_ang = 0;
            LHR_comp_ang = 0;
        }

    }
    else if(_t_foot_now < t_up + t_half_dsp + t_down - 0.5*dt){                  // foot down, 5th order trajectory down

        double compAngle = (-angleMax/t_down)*(_t_foot_now - (t_half_dsp + t_up)) + angleMax;

        if(_swingFoot == 1){ // Left Foot
            RHR_comp_ang = compAngle*(-1);
        }
        else if(_swingFoot == -1){ // Right Foot
            LHR_comp_ang = compAngle;
        }
        else{
            RHR_comp_ang = 0;
            LHR_comp_ang = 0;
        }
    }

    else
    {
        RHR_comp_ang = 0;
        LHR_comp_ang = 0;
    }

    return vec4(RHR_comp_ang, LHR_comp_ang ,0);

}

double StepTimeLimiter(double _step_time_gap, double _step_time_modi_old, double _dT, double _min_step_time){
    double step_time_modi = 0;

    step_time_modi = _step_time_gap;

    if(_dT >= _min_step_time){
        return step_time_modi;
    }
    else{ //if step time remains less than min_step_time, no time modification
        return _step_time_modi_old;
    }
}

vec3 CP_eos_Limitor(vec3 _CP_eos_gap, char swingFoot, double Foot_ori_x, double Foot_ori_y, vec3 _pRF_old, vec3 _pLF_old, double _dT){
    double safe_offset_y = 0.2;
    double reachable_dist = 0.5;
    double Max_vel = 1;

    vec3 CP_eos_modi = _CP_eos_gap;
    CP_eos_modi.z = 0;


    if(swingFoot == 1){ // Left Foot
        // Y direction position
        if(Foot_ori_y + CP_eos_modi.y < _pRF_old.y + safe_offset_y) CP_eos_modi.y = _pRF_old.y + safe_offset_y - Foot_ori_y;
        if(Foot_ori_y + CP_eos_modi.y > _pRF_old.y + reachable_dist) CP_eos_modi.y = _pRF_old.y + reachable_dist - Foot_ori_y;

        // X direction position
        if(Foot_ori_x + CP_eos_modi.x > _pRF_old.x + reachable_dist) CP_eos_modi.x = _pRF_old.x + reachable_dist - Foot_ori_x;
        if(Foot_ori_x + CP_eos_modi.x < _pRF_old.x - reachable_dist) CP_eos_modi.x = _pRF_old.x - reachable_dist - Foot_ori_x;

        // Y Velocity
        double diff_y = Foot_ori_y + CP_eos_modi.y - _pLF_old.y;
        if(diff_y/_dT > Max_vel){
            double tempY = diff_y - Max_vel*_dT;

            CP_eos_modi.y = CP_eos_modi.y - tempY;
        }

        // X Velocity
        double diff_x = Foot_ori_x + CP_eos_modi.x - _pLF_old.x;
        if(diff_x/_dT > Max_vel){
            double tempX = diff_x - Max_vel*_dT;

            CP_eos_modi.x = CP_eos_modi.x - tempX;
        }

        return CP_eos_modi;

    }
    else if(swingFoot == -1){
        // Y direction
        if(Foot_ori_y + CP_eos_modi.y > _pLF_old.y - safe_offset_y) CP_eos_modi.y = _pLF_old.y - safe_offset_y - Foot_ori_y;
        if(Foot_ori_y + CP_eos_modi.y < _pLF_old.y - reachable_dist) CP_eos_modi.y = _pLF_old.y - reachable_dist - Foot_ori_y;

        // X direction
        if(Foot_ori_x + CP_eos_modi.x > _pLF_old.x + reachable_dist) CP_eos_modi.x = _pLF_old.x + reachable_dist - Foot_ori_x;
        if(Foot_ori_x + CP_eos_modi.x < _pLF_old.x - reachable_dist) CP_eos_modi.x = _pLF_old.x - reachable_dist - Foot_ori_x;

        // Y Velocity
        double diff_y = Foot_ori_y + CP_eos_modi.y - _pRF_old.y;
        if(diff_y/_dT > Max_vel){
            double tempY = diff_y - Max_vel*_dT;

            CP_eos_modi.y = CP_eos_modi.y - tempY;
        }

        // X Velocity
        double diff_x = Foot_ori_x + CP_eos_modi.x - _pRF_old.x;
        if(diff_x/_dT > Max_vel){
            double tempX = diff_x - Max_vel*_dT;

            CP_eos_modi.x = CP_eos_modi.x - tempX;
        }

        return CP_eos_modi;
    }
    return CP_eos_modi;
}

vec3 Fifth_trajectory(double _t_now, double _T, vec3 _x_dx_ddx_old, vec3 _x_dx_ddx){

    double st_5,st_4,st_3,st_2,st_1;
    st_1 = _t_now;
    st_2 = st_1*st_1;
    st_3 = st_2*st_1;
    st_4 = st_3*st_1;
    st_5 = st_4*st_1;

    VectorNd Params = calc_5th(_t_now - dt, _T + dt, _x_dx_ddx_old, _x_dx_ddx);
    double x = Params(0)*st_5 + Params(1)*st_4 + Params(2)*st_3 + Params(3)*st_2 + Params(4)*st_1 + Params(5);
    double dx = 5*Params(0)*st_4 + 4*Params(1)*st_3 + 3*Params(2)*st_2 + 2*Params(3)*st_1 + Params(4);
    double ddx = 20*Params(0)*st_3 + 12*Params(1)*st_2 + 6*Params(2)*st_1 + 2*Params(3);

    vec3 x_dx_ddx = vec3(x, dx, ddx);

    return x_dx_ddx;

}

double HB_sign(double _a){
    if(_a >=0) return 1.0;
    else return -1.0;
}

mat3 inverse_HB(mat3 _M){
    double DET = (_M.m01*_M.m12-_M.m02*_M.m11)*_M.m20 + (_M.m02*_M.m10-_M.m00*_M.m12)*_M.m21 + (_M.m00*_M.m11-_M.m01*_M.m10)*_M.m22;

    double div = 1.0f/DET;
    double b00 = - (_M.m12*_M.m21-_M.m11*_M.m22)*div;
    double b01 =   (_M.m02*_M.m21-_M.m01*_M.m22)*div;
    double b02 = - (_M.m02*_M.m11-_M.m01*_M.m12)*div;
    double b10 =   (_M.m12*_M.m20-_M.m10*_M.m22)*div;
    double b11 = - (_M.m02*_M.m20-_M.m00*_M.m22)*div;
    double b12 =   (_M.m02*_M.m10-_M.m00*_M.m12)*div;
    double b20 = - (_M.m11*_M.m20-_M.m10*_M.m21)*div;
    double b21 =   (_M.m01*_M.m20-_M.m00*_M.m21)*div;
    double b22 = - (_M.m01*_M.m10-_M.m00*_M.m11)*div;

    mat3 M_inv;
    M_inv.m00 = b00; M_inv.m01 = b01; M_inv.m02 = b02;
    M_inv.m10 = b10; M_inv.m11 = b11; M_inv.m12 = b12;
    M_inv.m20 = b20; M_inv.m21 = b21; M_inv.m22 = b22;

    return M_inv;
}

quat inverse_HB(quat _Q){
    quat Q_inv;

    Q_inv.w = _Q.w;
    Q_inv.x = -_Q.x;
    Q_inv.y = -_Q.y;
    Q_inv.z = -_Q.z;

    return Q_inv;
}


vec4 Last_toe_Next_heel(char _last_RL, vec3 _last_Fpos, quat _last_Fquat, vec3 _next_Fpos, quat _next_Fquat){

    double Foot_length = 0.18;
    vec3 Last_toe, Next_heel;
    vec3 Local_last_to_next_Fpos;


    //// Find Local Coordinate

    if(_last_RL == -1){ //last foot is Right

        Local_last_to_next_Fpos = global2local_vec(_last_Fquat,_next_Fquat,_next_Fpos - _last_Fpos);
    }
    else if(_last_RL == 1){ //last foot is left

        Local_last_to_next_Fpos = global2local_vec(_next_Fquat, _last_Fquat, _next_Fpos - _last_Fpos);
    }
    else{
        return vec4(_last_Fpos.x, _last_Fpos.y, _next_Fpos.x, _next_Fpos.y);
    }


    //// Find Last Toe & next Heel Position

    if(Local_last_to_next_Fpos.x > 0.05 && Local_last_to_next_Fpos.x < Foot_length){  // step length is less than foot length && forward walking

        Last_toe = _last_Fpos + mat3(_last_Fquat)*vec3(Local_last_to_next_Fpos.x/2,0,0);

        Next_heel = _next_Fpos + mat3(_next_Fquat)*vec3(-Local_last_to_next_Fpos.x/2,0,0);
    }

    else if(Local_last_to_next_Fpos.x >= Foot_length){  // step length is longer than foot length && forward walking

        Last_toe = _last_Fpos + mat3(_last_Fquat)*vec3(Foot_length/2,0,0);

        Next_heel = _next_Fpos + mat3(_next_Fquat)*vec3(-Foot_length/2,0,0);
    }

    else if(Local_last_to_next_Fpos.x < -0.05 && Local_last_to_next_Fpos.x > -Foot_length){  // step length is less than foot length && backrward walking

        Last_toe = _last_Fpos + mat3(_last_Fquat)*vec3(Local_last_to_next_Fpos.x/2,0,0);

        Next_heel = _next_Fpos + mat3(_next_Fquat)*vec3(-Local_last_to_next_Fpos.x/2,0,0);

    }

    else if(Local_last_to_next_Fpos.x <= -Foot_length){  // step length is longer than foot length && backward walking

        Last_toe = _last_Fpos + mat3(_last_Fquat)*vec3(-Foot_length/2,0,0);

        Next_heel = _next_Fpos + mat3(_next_Fquat)*vec3(+Foot_length/2,0,0);

    }

    else{
        Last_toe = _last_Fpos;

        Next_heel = _next_Fpos;
    }

    return vec4(Last_toe.x, Last_toe.y, Next_heel.x, Next_heel.y);

}


