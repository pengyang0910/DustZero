#include <iostream>
#include <memory>
#include <termios.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <fstream>
#include <memory.h>
#include <sstream>
#include "stdio.h"
#include <openssl/md5.h>
#include <vector>

/* >>>> log <<<< */
#include <ctime>
#include <stdlib.h>

#include "xini.h"
#include "xlog.h"


std::string md5sum(const std::string &str) {
 
    std::string md5;
    MD5_CTX ctx;
    MD5_Init(&ctx);
    MD5_Update(&ctx, str.c_str(), str.size());
    unsigned char digest[MD5_DIGEST_LENGTH];
    MD5_Final(digest, &ctx);
    char hex[35];
    memset(hex, 0, sizeof(hex));
    for (int i = 0; i < MD5_DIGEST_LENGTH; ++i){
        sprintf(hex + i * 2, "%02x", digest[i]);
    }
    md5 = std::string(hex);
    return md5;
}

bool md5sumFile(const std::string &file,std::string & result) {
 
    FILE *inFile = fopen (file.c_str(), "rb");
	if(inFile != NULL) {
		MD5_CTX mdContext;
		int bytes;
		unsigned char data[1024];
		unsigned char digest[MD5_DIGEST_LENGTH];
		MD5_Init (&mdContext);
		while ((bytes = fread (data, 1, 1024, inFile)) != 0)
			MD5_Update (&mdContext, data, bytes);
		MD5_Final (digest,&mdContext);

		char hex[35];
		memset(hex, 0, sizeof(hex));
		for (int i = 0; i < MD5_DIGEST_LENGTH; ++i){
			sprintf(hex + i * 2, "%02x", digest[i]);
		}
		result =  std::string(hex);

		return true;
	}
	else return false;
}

bool loadMd5Data(std::string file, std::vector<std::pair<std::string, std::string>>& fileNames)
{	
	FILE *inFile = fopen (file.c_str(), "r+");
	if(inFile == NULL)
	{
		printf("open error");
		exit(-1);
	}
	
	std::string s, str, s1, s2, s3;
	fileNames.clear();
	int cnt = 0;

	char linebuffer[512] = {0};
	char buffer1[512] = {0}; 
	char buffer2[512] = {" = "};

	int line_len = 0;
	int len = 0;
	int res;

	while(fgets(linebuffer, 512, inFile))
	{
		//line_len = strlen(linebuffer);
		//len += line_len;	
		sscanf(linebuffer, "%12s", buffer1);
		std::istringstream in(linebuffer);
		in >> s1>>s2;

		std::pair<std::string, std::string> curr_file;
		curr_file.first =  s1;
		curr_file.second = s2;//.c_str();
		
		fileNames.push_back(curr_file);

		cnt++;
		printf("cnt is %d  str is  %s \n",cnt,s1.c_str());
	}

	fclose(inFile);

	printf("file cnt is %d \n",cnt);
	if (cnt==0)
	{
		return false;
	}
	else return true;
}

bool checkMd5(std::vector<std::pair<std::string, std::string>> fileNames)
{
	for (size_t i = 0; i < fileNames.size(); i++)
	{
		std::pair<std::string, std::string> curr_file = fileNames[i];
		std::string file = "/app/fj212/bin/"+curr_file.second;
		std::string final;
		bool issucuss = md5sumFile(file,final);
		if (issucuss ==false || curr_file.first != final)
		{
			return false;
		}
		printf("%d is %s %s \n",i,file.c_str(),final.c_str());
	}

	return true;
}

void Init()
{
	std::vector<std::pair<std::string, std::string>> fileNames;
	bool flag = loadMd5Data("/app/fj212/bin/md5.txt",fileNames);
	if (flag)
	{
		bool flag1 = true;//checkMd5(fileNames);
		if (flag1)
		{
			printf("-----------  startFj212 ------------------\n");
			system("/app/Upgrade/startFj212.sh");
		}
		else
		{	
			printf("start to unzip \n");
			system("unzip -o  /userdata/xt212.zip  -d /app/fj212 && sync");
			
			bool flag2 = loadMd5Data("/app/fj212/bin/md5.txt",fileNames);
			if (flag2)
			{
				bool flag3 = checkMd5(fileNames);
				if (flag3)
				{
					system("/app/Upgrade/startFj212.sh");
				}
				else printf("update.tar is error \n");
			}
			else 
			{
				printf("load file  is error \n");
			}
			
		}	
	}
	else
	{
		//system("unzip -o  /mnt/UDISK/fj212/xt212.zip -d /mnt/UDISK/fj212/");
		system("unzip -o  /userdata/xt212.zip -d /app/fj212/ && sync");
		//system("rm /mnt/UDISK/fj212/resource/upgrade.tar.gz");
		bool flag2 = loadMd5Data("/app/fj212/bin/md5.txt",fileNames);
		if (flag2)
		{
			bool flag3 = checkMd5(fileNames);
			if (flag3)
			{
				system("/app/Upgrade/startFj212.sh");
			}
			else printf("update.tar is error \n");
		}
		else 
		{
			printf("load file  is error \n");
		}
	}
	//system("frpc -c /userdata/cfg/frpc.ini  &");
}


int main(int argc, char **argv)
{
	Init();

	while (1)
	{
		usleep(50000);
	}
	
	return 0;
}

