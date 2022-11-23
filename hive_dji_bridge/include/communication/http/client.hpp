#pragma once

#include <vector>
#include <iostream>
#include <curl/curl.h>
#include <communication/http/http_response.hpp>

class HttpClient {
public:
    HttpClient() noexcept
      : _headers(nullptr)
      , _timeout(30L) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    }

    void addHeaders(const std::vector<std::string>& slist) {
        for (const auto& list : slist) {
            _headers = curl_slist_append(_headers, list.data());
        }
    }

    void setTimeout(long timeout) noexcept {
        _timeout = timeout;
    }

    HttpResponse sendRequest(const char*, const char*, const char*) const;

    HttpResponse POST(const char* url, const char* data) const {
        return sendRequest("POST", url, data);
    }

    HttpResponse PATCH(const char* url, const char* data) const {
        return sendRequest("PATCH", url, data);
    }

    HttpResponse GET(const char* url) const {
        return sendRequest("GET", url, nullptr);
    }

    void cleanupHeaders() {
        curl_slist_free_all(_headers);
        _headers = nullptr;
    }

    ~HttpClient() {
        if (_headers != nullptr) {
            cleanupHeaders();
        }
        curl_global_cleanup();
    }

private:
    curl_slist* _headers;
    long _timeout;
};