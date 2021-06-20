import { Component, ViewChild, OnInit, ElementRef } from '@angular/core';
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
    mode: 'dynamic',
    catchDistance: 100,
    color: 'purple',
    size: 200,
  };

  semiOutputData: JoystickOutputData;

  recStatus = false;

  constructor(public directionService: DirectionService) {
  }

  onMotorMove(event: JoystickEvent) {
    this.semiOutputData = event.data;
    this.directionService.receiveMotorMoveEvent(event);
  }

  onSteerMove(event: JoystickEvent) {
    this.semiOutputData = event.data;
    this.directionService.receiveSteerMoveEvent(event);
  }

  onMotorEnd(event: JoystickEvent) {
    this.directionService.receiveMotorEndEvent();
  }

  onSteerEnd(event: JoystickEvent) {
    this.directionService.receiveSteerEndEvent();
  }

  getRecStatusText(): string {
    return this.recStatus ? 'REC' : 'Waiting'
  }

  onClickRecBtn(): void {
    this.recStatus = !this.recStatus;
    this.directionService.sendRecCmd(this.recStatus);
  }

}
