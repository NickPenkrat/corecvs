#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include "kde.h"

double corecvs::kde::get_min(int x){
    default_bandwidth(x);
    return(min_map[x]-(extension*default_bandwidth_map[x]));
}

double corecvs::kde::get_max(int x){
    default_bandwidth(x);
    return(max_map[x]+(extension*default_bandwidth_map[x]));
}

void corecvs::kde::default_bandwidth(int curr_var){

    if(!count_map[curr_var]){
        std::cout << "No data!" << std::endl;
        exit(1);
    }
    double x  = sum_x_map[curr_var]/count_map[curr_var];
    double x2 = sum_x2_map[curr_var]/count_map[curr_var];
    double sigma = sqrt(x2 - (x*x));
    double b = sigma*(pow((3.0*count_map[curr_var]/4.0),(-1.0/5.0)));

    default_bandwidth_map[curr_var] = b;
}

double corecvs::kde::curvature(double x, double w, std::vector<double>& data, int curr_var){

    double y = 0.0;
    for(auto it = data.begin(); it != data.end(); it++){
        y += gauss_curvature(x,*it,w);
    }
    return(y/count_map[curr_var]);

}

double corecvs::kde::stiffness_integral(double w, double mn, double mx, std::vector<double>& data, int curr_var){

    double eps = 1e-4;
    double n = 1;
    double dx = (mx-mn)/n;
    double curv_mx = curvature(mx,w,data,curr_var);
    double curv_mn = curvature(mn,w,data,curr_var);
    double yy = 0.5*((curv_mx*curv_mx)+(curv_mn*curv_mn))*dx;
    double maxn = (mx-mn)/sqrt(eps);

    maxn = maxn > 2048 ? 2048 : maxn;

    for(int n = 2; n <= maxn; n *= 2){
        dx /= 2.0;
        double y = 0.0;
        for(int i = 1; i <= n-1; i +=2){
            curv_mn = pow(curvature(mn + i*dx, w, data, curr_var),2);
            y += curv_mn;
        }
        yy = 0.5*yy + y*dx;
        if(n > 8 && abs(y*dx-0.5*yy) < eps*yy){
            break;
        }
    }
    return(yy);
}

double corecvs::kde::optimal_bandwidth_equation(double w, double min, double max, std::vector<double>& data, int curr_var){

    double alpha = 1.0/(2.0*sqrt(M_PI));
    double sigma = 1.0;
    double n = count_map[curr_var];
    double q = stiffness_integral(w,min,max,data,curr_var);
    return w - pow(((n*q*pow(sigma,4))/alpha ),(-1.0/5.0));
}

void corecvs::kde::calc_bandwidth(){
    for(int curr_var = 0; curr_var < data_matrix.size(); curr_var++){
        if(bandwidth_map[curr_var] == -1.0){

            if(!count_map[curr_var]){
                std::cout << "No data!" << std::endl;
                exit(1);
            }

            double eps = 1e-03;

            default_bandwidth(curr_var);
            double x0 = default_bandwidth_map[curr_var]/count_map[curr_var];
            double x1 = 2*default_bandwidth_map[curr_var];
            double y0 = optimal_bandwidth_equation(x0,min_map[curr_var],max_map[curr_var],data_matrix[curr_var], curr_var);
            double y1 = optimal_bandwidth_equation(x1,min_map[curr_var],max_map[curr_var],data_matrix[curr_var], curr_var);

            double x = 0.0, y = 0.0;
            int i = 0;

            while(abs(x0 - x1) > eps*x1){
                i += 1;
                x = (x0 + x1 )/2;
                y = optimal_bandwidth_equation(x,min_map[curr_var],max_map[curr_var],data_matrix[curr_var], curr_var);

                if(abs(y) < eps*y0){
                    break;
                }
                if(y * y0 < 0){
                    x1 = x;
                    y1 = y;
                }else{
                    x0 = x;
                    y0 = y;
                }
            }
            bandwidth_map[curr_var] = x;

        }
    }
}

double corecvs::kde::get_bandwidth(int x){
    return(bandwidth_map[x]);
}

double corecvs::kde::gauss_curvature(double x, double m, double s){
    double z = (x - m)/s;
    return ((z*z) - 1.0)*gauss_pdf(x,m,s)/(s*s);
}


double corecvs::kde::gauss_pdf(double x, double m, double s){
    double z = (x - m)/s;
    return exp(-0.5*z*z)/(s*sqrt( 2.0*M_PI));
}

double corecvs::kde::pdf(double x){
    std::vector<double> tmp;
    tmp.push_back(x);
    return(pdf(tmp));
}

double corecvs::kde::pdf(double x, double y){
    std::vector<double> tmp;
    tmp.push_back(x);
    tmp.push_back(y);
    return(pdf(tmp));
}

double corecvs::kde::pdf(std::vector<double>& data){
    double d = 0.0;
    for(int i = 0; i < data_matrix[0].size(); i++){
        double a = 1.0;
        for(int curr_var = 0; curr_var < data_matrix.size(); curr_var++){
            a *= gauss_pdf(data[curr_var],data_matrix[curr_var][i],bandwidth_map[curr_var]);
        }
        d += a;
    }
    return(d/count_map[0]);
}

void corecvs::kde::add_data(double x){
    std::vector<double> tmp;
    tmp.push_back(x);
    add_data(tmp);
}

void corecvs::kde::add_data(double x, double y){
    std::vector<double> tmp;
    tmp.push_back(x);
    tmp.push_back(y);
    add_data(tmp);
}

void corecvs::kde::add_data(std::vector<double>& x){

    if(!data_matrix.size()){
        for(int i = 0; i < x.size(); i++){
            std::vector<double> tmp;
            tmp.push_back(x[i]);
            data_matrix.push_back(tmp);
            sum_x_map[i] = x[i];
            sum_x2_map[i] = x[i]*x[i];
            count_map[i] = 1;
            min_map[i] = x[i];
            max_map[i] = x[i];
            bandwidth_map[i] = -1.0;
        }
    }else{
        if(x.size() != data_matrix.size()){
            std::cout << "Number of variables doesn't match!" << std::endl;
        }else{
            for(int i = 0; i < x.size(); i++){
                std::cout << "Data I\t" << data_matrix[i][i]  << std::endl;
                std::cout << "X I\t" << x[i]  << std::endl;
                data_matrix[i].push_back(x[i]);
                std::cout << "Data I + 1\t" << data_matrix[i][i+1]  << std::endl;
                std::cout << "Sum I\t" << sum_x_map[i]  << std::endl;
                std::cout << "X I\t" << x[i]  << std::endl;
                sum_x_map[i] += x[i];
                std::cout << "Sum I + 1\t" << sum_x_map[i+1]  << std::endl;
                sum_x2_map[i] += x[i]*x[i];
                count_map[i]++;
                std::cout << "min_map B I\t" << min_map[i]  << std::endl;
                std::cout << "max_map B I\t" << max_map[i]  << std::endl;
                min_map[i] = x[i] < min_map[i] ? x[i] : min_map[i];
                max_map[i] = x[i] > max_map[i] ? x[i] : max_map[i];
                std::cout << "min_map A I\t" << min_map[i]  << std::endl;
                std::cout << "max_map A I\t" << max_map[i]  << std::endl;
                bandwidth_map[i] = -1.0;


            }
        }
    }
}

void corecvs::kde::calc_pdf(int testPointCountX, int testPointCountY)
{
    ndX = testPointCountX;
    ndY = testPointCountY;

//    calc_bandwidth();

    double min_x = get_min(0);
    double max_x = get_max(0);
    double min_y = get_min(1);
    double max_y = get_max(1);

    std::cout << "min x,y = " << min_x << "," << min_y << std::endl;
    std::cout << "max x,y = " << max_x << "," << max_y << std::endl;

    double x_increment = (max_x-min_x)/ndX;
    double y_increment = (max_y-min_y)/ndY;
    double y = min_y;
    double x = min_x;

    std::cout << "increment x,y = " << x_increment << "," << y_increment << std::endl;

    std::cout << "# bandwidth var 1: " << get_bandwidth(0) << std::endl;
    std::cout << "# bandwidth var 2: " << get_bandwidth(1) << std::endl;

    for(int i = 0; i < ndX; i++){
        x += x_increment;
        y = min_y;
        for(int j = 0; j < ndY; j++){
            y += y_increment;
            std::cout << "x,y pdf \t" << x << ", " << y << "\t" << pdf(x,y) << std::endl;
        }
    }
    std::cout << "Complete" << std::endl;
}
