/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-04-21 14:02:21
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-06-14 16:09:10
 * @FilePath: /xt212_navclean/XUtils/testini.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "XCfg/xini.h"
#include "math.h"
void loadIni()
{
    //testFrame();
    //printf("%s %s %s %s\r\n", \
    //typeid(int).name(), typeid(double).name(), typeid(bool).name(), typeid(bool*).name());
    ini::IniFile myIni;
    myIni.setMultiLineValues(false);
    myIni.load("test.ini");
    double d ;
    
    {
        d = myIni.GetProperty("Bar", "myDouble").as<double>();
        auto str = myIni.GetProperty("Bar", "myStr").as<std::string>();
        std::cout<<"myStr= "<<str<<std::endl;
        myIni["Nar"]["new"] = 3.3;
        //double yy = myIni["Bar"]["YY"].as<double>();
        //double kk = myIni["Bar"][myIni.GetKey("hex")].as<double>();
       // auto mmm = myIni["Bar"]["myDoubel"].as<bool>();
    }

    
    
    float k;
    //k = myIni["Foo"]["kkk"].as<double>();
    printf("before d = %f k = %f\r\n", d, k);
    myIni["Bar"]["myDouble"] = d+0.3;
    myIni.save("test.ini");
}


void test_xd1qFilter()
{
	std::vector<float> xd1qRawData;
	std::ifstream inf("data.txt", std::ios::in);

    std::string line;
    int seg_id = -1;
    while (std::getline(inf, line))
    {
        std::istringstream iss(line);
        float x,y,z;
        std::string type;
        iss >> x >> y >> z;
		float data = sqrt(x*x+y*y+z*z);
		xd1qRawData.push_back(data);
	}
	printf("xd1qRawData.size() = %d\r\n", xd1qRawData.size());

	for (int i = 0; i < xd1qRawData.size(); i++)
	{

		int j = i;
			for (; j < xd1qRawData.size(); j++)
			{
				/* code */
				if(fabs(xd1qRawData[i] - xd1qRawData[j]) < 0.01) // 1cm
				{
					j++;
				}
				else 
					break;
			}

			if(j - i > 8) // 0.15 *8 = 1.2 /deg
			{
				i = j; // valid
				
			}
			else
			{
				// invalid: set data = 0
				while (i<j)
				{
					/* code */
					xd1qRawData[i] = 0;
					i++;
					printf("set i=%d 0\r\n", i+1);
				}
			}
			

			// printf("raw i=%d dis = %d (%.3f, %.3f, %.3f) \r\n", i, hwInput.m_xd1q_points[i], xd1qData.x[i], xd1qData.y[i], xd1qData.z[i]);
			//  if(i%10 == 0)
			//  xlogPtr->Debug("xd1q-%d: %d\t", i, hwInput.m_xd1q_points[i]);
	}


}

int main(int argc, char **argv)
{

   test_xd1qFilter();
    //testIni();
    //loadIni();
    return 0;
}