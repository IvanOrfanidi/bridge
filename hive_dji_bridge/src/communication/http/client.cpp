#include <boost/assert.hpp>
#include <communication/http/client.hpp>

size_t curlWriteCallbackFuncStdString(void* contents, size_t size, size_t nmemb, std::string* string) {
    const size_t length = size * nmemb;
    try {
        string->append((char*)contents, length);
    } catch (std::bad_alloc& exception) { return 0; }
    return length;
}

HttpResponse HttpClient::sendRequest(const char* request, const char* url, const char* data) const {
    BOOST_ASSERT_MSG(request != nullptr, "HTTP request type must not be empty");
    BOOST_ASSERT_MSG(url != nullptr, "HTTP URL must not be empty");
    std::string response;
    CURL* session = curl_easy_init();
    if (session == nullptr) {
        return HttpResponse{};
    }

    curl_easy_setopt(session, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(session, CURLOPT_URL, url);
    curl_easy_setopt(session, CURLOPT_CUSTOMREQUEST, request);

    if (data != nullptr && data[0] != '\0') {
        curl_easy_setopt(session, CURLOPT_POSTFIELDS, data);
    }

    curl_easy_setopt(session, CURLOPT_HTTPHEADER, _headers);
    curl_easy_setopt(session, CURLOPT_TIMEOUT, _timeout);
    curl_easy_setopt(session, CURLOPT_WRITEFUNCTION, curlWriteCallbackFuncStdString);
    curl_easy_setopt(session, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(session);
    if (res != CURLE_OK) {
        curl_easy_cleanup(session);
        return HttpResponse{};
    }
    long httpCode = 0;
    res = curl_easy_getinfo(session, CURLINFO_RESPONSE_CODE, &httpCode);
    curl_easy_cleanup(session);

    return HttpResponse{res == CURLE_OK, httpCode, response};
}