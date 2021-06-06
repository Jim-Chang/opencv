import { Component, ViewChild } from '@angular/core';
import { JoystickEvent, NgxJoystickComponent } from 'ngx-joystick';
import { JoystickManagerOptions, JoystickOutputData } from 'nipplejs';
import { DirectionService } from './services/direction.service';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.sass']
})
export class AppComponent {
  title = 'joystick';

  @ViewChild('semiJoystick') semiJoystick: NgxJoystickComponent;

  semiOptions: JoystickManagerOptions = {
    mode: 'semi',
    catchDistance: 100,
    color: 'purple',
    size: 200,
  };

  semiOutputData: JoystickOutputData;


  constructor(private directionService: DirectionService) {
  }

  onMove(event: JoystickEvent) {
    this.semiOutputData = event.data;
    this.directionService.receiveMoveEvent(event);
  }

  onEnd(event: JoystickEvent) {
    this.directionService.receiveStopEvent();
  }

}
