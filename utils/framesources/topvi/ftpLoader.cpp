#include "ftpLoader.h"

#include <stdio.h>
#include <string.h>
#include <curl/curl.h>


FtpLoader::FtpLoader()
{

}

struct FtpFile {
  const char *filename;
  FILE *stream;
};

static size_t getFtpDataCallback(void *buffer, size_t size, size_t nmemb, void *stream)
{
  struct FtpFile *out=(struct FtpFile *)stream;
  if (out && !out->stream) {
    out->stream = fopen(out->filename, "wb");
    if (!out->stream) {
            fprintf(stderr, "Can't open file to write\n");
      return -1;
        }
  }
  return fwrite(buffer, size, nmemb, out->stream);
}

int FtpLoader::ftpGetTest() {
  CURL *curl;
  CURLcode res;

  struct FtpFile ftpfile = {
        "newFile.zip",
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
      fprintf(stderr, "Problems with curl library: %d\n", res);
    }
  }

  if (ftpfile.stream) fclose(ftpfile.stream);

  curl_global_cleanup();

  return res;
}
