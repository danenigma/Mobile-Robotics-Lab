 function encoderEventListener(handle,event)

          global encoderDataTimeStamp;
          encoderDataTimeStamp = double(event.Header.Stamp.Sec) + ...
		  double(event.Header.Stamp.Nsec)/1000000000.0;

  end