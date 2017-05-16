#include "ftpLoader.h"

#include <stdio.h>
#include <string.h>
#include <curl/curl.h>

#include "global.h"

struct FtpFile {
    const char *filename;
    FILE *stream;
};

static size_t getFtpDataCallback(void *buffer, size_t size, size_t nmemb, void *stream)
{
    struct FtpFile *out = (struct FtpFile *)stream;
    if (out && !out->stream) {
        out->stream = fopen(out->filename, "wb");
        if (!out->stream) {
            SYNC_PRINT(("Can't open file %s to write\n", out->filename));
            return -1;
        }
    }
    return fwrite(buffer, size, nmemb, out->stream);
}

int FtpLoader::makeTest() {
    CURL *curl;
    CURLcode res;

    struct FtpFile ftpfile = {
          "testFile.zip",
          NULL
    };

    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl = curl_easy_init();
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, "ftp://speedtest.tele2.net/1KB.zip");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, getFtpDataCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ftpfile);
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        if (CURLE_OK != res) {
            SYNC_PRINT(("Problems with curl library: %d\n", res));
        }
    }

    if (ftpfile.stream) fclose(ftpfile.stream);

    curl_global_cleanup();

    return res;
}

int FtpLoader::init(string _addr, string reply) {
    setAddr(_addr);
    SYNC_PRINT(("FtpLoader::init() called for %s with links = %s\n", _addr.c_str(), reply.c_str()));

    size_t pos = 0;
    string _link = "";
    this->linksNumber = 0;
    do {
      pos = reply.find_first_of(",");
      _link = reply.substr(0, pos);
      if (_link != "") {
          SYNC_PRINT(("links[%d] = %s\n", linksNumber, _link.c_str()));
          _link = _addr + _link;
          this->links.push_back(_link);
          linksNumber++;
      }
      reply = reply.substr(pos + 1);
      SYNC_PRINT(("[%d] rest of reply: ", linksNumber, reply.c_str()));
    } while (pos != std::string::npos);

    if (linksNumber > 0) {
        inited = true;
    }
    return inited;
}

void FtpLoader::setAddr(string _addr) {
    if (_addr != "") this->addr = _addr;
}

void FtpLoader::setOutput(string _outputDir) {
    if (_outputDir != "") this->outputDir = _outputDir;
}

int FtpLoader::getFile(string link)
{
    string fname = link.substr(link.find_last_of("/\\") + 1);
    SYNC_PRINT(("FtpLoader::getFile() %s\n", fname.c_str()));
    if (!this->outputDir.empty()) {
        fname = this->outputDir + "/" + fname;
    }

    struct FtpFile ftpfile = {
          fname.c_str(),
          NULL
    };

    curl_global_init(CURL_GLOBAL_DEFAULT);

    CURLcode res;
    CURL *curl = curl_easy_init();
    if (curl != nullptr)
    {
        curl_easy_setopt(curl, CURLOPT_URL, link.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, getFtpDataCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ftpfile);
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        res = curl_easy_perform(curl);
        fflush(stderr);
        fflush(stdout);
        curl_easy_cleanup(curl);
        if (CURLE_OK != res) {
            SYNC_PRINT(("Problems with curl library: %d\n", res));
        }
    }

    if (ftpfile.stream != nullptr) {
        fclose(ftpfile.stream);
        activeFile = fname;
    }

    curl_global_cleanup();
    return CURLE_OK == res ? 0 : -1;
}
