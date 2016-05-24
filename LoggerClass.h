#include <QueueList.h>
#include <HardwareSerial.h>
#include <stdlib.h>

#define EVENT_DESCRIPTION_MAX_CHARS (64)

struct Event
{
  EVENT_TYPE type;
  STATE currentState;
  STATE nextState;
  QueueList<int> data;
  char description[EVENT_DESCRIPTION_MAX_CHARS];
  unsigned long timestamp;
};

struct EventQuery
{
  EVENT_TYPE t;
};

class LoggerClass
{
  private:
  volatile bool _is_locked;
  
  bool IsLocked() const 
  {
    return _is_locked;
  }
    
  void Lock()
  {
    _is_locked = true;
  }
  
  void ReleaseAccess()
  {
    _is_locked = false;
  }
  
  QueueList<Event> _log;
  QueueList<Event> _to_add;

  void PrintEvent(HardwareSerial & s, Event & e)
  {
    s.println(e.description);
  }
  
  bool CanAccess()
  {
    bool result = !IsLocked();
    if (result)
    {
      Lock();
    }
    return result;
  }

  //return true if successful
  bool TakeFromAddList(Event & out_event)
  {
    if (CanAccess())
    {
      out_event = _to_add.pop();
      ReleaseAccess();
      return true;
    }
    return false;
  }
  
  bool AddToLog(Event e)
  {
    if (CanAccess())
    {
      _log.push(e);
      ReleaseAccess();
      return true;
    }
   return false; 
  }
  
  public:
  //TODO: implement this
  bool GetEvents(EventQuery query, QueueList<Event> & out_event_list)
  {
    //WaitForAccess();
    //QueueList<Event> & result = _log;
    //ReleaseAccess();  
    return false;
  }

  
  bool LogEvent(Event event)
  {
    Serial.println("attempting to log event...waiting for access...");
    if (CanAccess())
    {
      _to_add.push(event); 
      Serial.println("done logging event...releasing access...");
      ReleaseAccess();
      return true;
    }
    return false;
  }
  
  //called by periodic thread
  void ProcessLog(HardwareSerial & s)
  {
    s.println("logger thread - ProcessLog");
    s.print("size of to_add: ");
    s.print(_to_add.count());
    s.print("\n");
    for (int i = 0; i < _to_add.count(); i++)
    {
      Event e = {};
      if (TakeFromAddList(e))
      {
        AddToLog(e);
        PrintEvent(s,e);
      } 
    }
  }

   LoggerClass()
  {
    _is_locked = false;
    // add one event by default
    Event newEvent = {};
    newEvent.type = LIGHT_SENSOR_FALLING_EDGE_EVENT;
    newEvent.currentState = WAITING_FOR_BOUNCE;
    newEvent.nextState = DEBOUNCING;
    newEvent.data = {};
    snprintf(newEvent.description, EVENT_DESCRIPTION_MAX_CHARS, "LIGHT_SENSOR_FALLING_EDGE_EVENT. Transitioning to debouncing state.");
    newEvent.timestamp = millis();
    //_to_add.push_back(newEvent);
  }
};
