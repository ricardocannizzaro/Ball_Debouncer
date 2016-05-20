#include <SoftwareSerial.h>
#include <stdlib.h>

#define EVENT_DESCRIPTION_MAX_CHARS (64)

template <class T>
class Queue {
  T * _data;
  T * _start;
  int _numEvents;
  int _size;
  public:
  Queue()
  {
    _data = (T*)malloc(_size * sizeof(T));
    _size = 32;
    _start = _data;
    _numEvents = 0;
  }
  ~Queue()
  {
    free(_start); // destroy from the start of the array
  } 
  void push_back(T event)
  {
    if (_numEvents >= _size)
    {
      _size = _size * 2;
      _data = (T* )realloc(_data, _size * sizeof(T));
      _start = _data;
    }
    _data[_numEvents] = event;
    _numEvents++;
  }
  T back()
  {
    if (_size > 0)
    {
      return _data[_numEvents-1];
    }
    else
    {
      T result = {};
      return result;
    }
  }
  void pop_back()
  {
    _numEvents--;
  }
  T front()
  {
    return _data[0];
  }
  void pop_front()
  {
    _data++;
    _numEvents--;
  }
  unsigned int size(){return _size;}
};

enum EventType{};
enum StateType{};
  
struct Event
{
  EventType type;
  StateType currentState;
  StateType nextState;
  Queue<int> data;
  char description[EVENT_DESCRIPTION_MAX_CHARS];
  unsigned long timestamp;
};

struct EventQuery
{
  EventType t;
};

class Logger
{
  private:
  volatile bool _is_locked;
  
  bool IsLocked() const {return _is_locked;}
  void Lock(){_is_locked = true;}
  void ReleaseAccess(){_is_locked = false;}
  
  Queue<Event> _log;
  Queue<Event> _to_add;


  void PrintEvent(SoftwareSerial & s, Event & e)
  {
    s.println(e.description);
  }
  
  
  void WaitForAccess()
  {
    while(IsLocked()){};
    Lock();
  }
  
  Event TakeFromAddList()
  {
    WaitForAccess();
    Event result = _to_add.back();
    _to_add.pop_back();
    ReleaseAccess();
    return result;
  }
  
  void AddToLog(Event & e)
  {
    WaitForAccess();
    _log.push_back(e);
    ReleaseAccess();
  }
  public:
  Queue<Event> GetEvents(EventQuery & query)
  {
    WaitForAccess();
    Queue<Event> & result = _log;
    ReleaseAccess();  
    return result;
  }
  
  void LogEvent(Event & event)
  {
    WaitForAccess();
    _to_add.push_back(event); 
    ReleaseAccess();
  }
  
  //called by periodic thread
  void ProcessLog(SoftwareSerial & s)
  {
    s.println("logger thread - ProcessLog");
    for (int i = 0; i < _to_add.size(); i++)
    {
      Event e = TakeFromAddList();
      AddToLog(e);
      PrintEvent(s,e);
    }
  }
};