#ifndef __ZSS_PLUGIN_H__
#define __ZSS_PLUGIN_H__
#include <iostream>
#include <string>
#include <thread>
#include <list>
#include <map>
#include <shared_mutex>
#include <cstring>
#include <mutex>
#include <condition_variable>
class Semaphore {
public:
    Semaphore(long count = 0)
        : count_(count) {
    }
    Semaphore(const Semaphore& s):count_(0){
    }

    void Signal(unsigned int c = 1) {
        std::unique_lock<std::mutex> lock(mutex_);
        count_=c;
        cv_.notify_one();
    }

    void Wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [=] { return count_ > 0; });
        --count_;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    long count_;
};
class ZSData{
public:
    ZSData():_size(0),_capacity(0),_data(nullptr){}
    ZSData(const ZSData& data):_size(0),_capacity(0),_data(nullptr){
        store(data);
    }
    ~ZSData(){
        if(_capacity > 0){
            free(_data);
        }
    }
    virtual int size() const {
        std::shared_lock<std::shared_mutex> lock(_mutex);
        return _size;
    }
    virtual void copy(void* data){
        std::shared_lock<std::shared_mutex> lock(_mutex);
        memcpy(data,_data,_size);
    }
    virtual const void* data() const { return _data; }
    virtual void store(const ZSData& data){
        store(data.data(),data._size);
    }
    virtual void store(const void* const data,unsigned long size){
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if(size > _capacity){
            if(_data != nullptr){
                free(_data);
            }
            _data = malloc(size);
            _capacity = size;
        }
        _size = size;
        memcpy(_data,data,size);
    }
protected:
    unsigned long _size;
    int _capacity;
    void* _data;
    mutable std::shared_mutex _mutex;
};
class ZSSemaData:public ZSData,public Semaphore{

};
class ZSPlugin{
public:
    ZSPlugin(){}
    virtual ~ZSPlugin(){}
	virtual void run()=0;
    virtual void publish(const std::string&& msg,const void* data,const unsigned long size) final{
        auto it = _subscribers.find(msg);
        if (it != _subscribers.end()){
            for(auto p:_subscribers[msg]){
                p->store(msg,data,size);
            }
        }
	}
    virtual void receive(const std::string& msg,ZSData& data) final{
        auto it = _databox.find(msg);
        if (it == _databox.end()){
            std::cerr << "ERROR : didn't DECLARE to RECEIVE this kind of message, check your message type : " << msg << std::endl;
            return;
        }
        _databox[msg].Wait();
        data.store(_databox[msg]);
    }
    virtual void connect(ZSPlugin* p,const std::string& msg) final{
        auto it = _subscribers.find(msg);
        if (it == _subscribers.end())
            _subscribers[msg] = {};
        _subscribers[msg].push_back(p);
	}
    virtual void store(const std::string& msg,const void* data,const unsigned long size) final{
        auto it = _databox.find(msg);
        if (it == _databox.end()){
            std::cerr << "ERROR : didn't DECLARE to STORE this kind of message, check your message type : " << msg << std::endl;
            return;
        }
        _databox[msg].store(data,size);
        _databox[msg].Signal();
    }
    virtual void declare(const std::string& msg) final{
        auto it = _databox.find(msg);
        if (it != _databox.end()){
            std::cerr << "ERROR : REDECLARE, check your message type : " << msg << std::endl;
            return;
        }
        _databox.insert(std::pair<std::string,ZSSemaData>(msg,ZSSemaData()));
    }
private:
    std::map<std::string,std::list<ZSPlugin*>> _subscribers = {};
    std::map<std::string,class ZSSemaData> _databox = {};
};

#endif // __ZSS_PLUGIN_H__
