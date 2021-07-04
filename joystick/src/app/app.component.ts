import { Component, ViewChild, OnInit, ElementRef, Inject } from '@angular/core';
import { DOCUMENT } from '@angular/common';
import { JoystickEvent, NgxJoystickComponent } from 'ngx-joystick';
import { JoystickManagerOptions, JoystickOutputData } from 'nipplejs';
import { DirectionService } from './services/direction.service';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.sass']
})
export class AppComponent implements OnInit {
  title = 'joystick';

  @ViewChild('semiJoystick') semiJoystick: NgxJoystickComponent;
  private docElem: any;

  semiOptions: JoystickManagerOptions = {
    mode: 'dynamic',
    catchDistance: 100,
    color: 'purple',
    size: 200,
  };

  semiOutputData: JoystickOutputData;

  recStatus = false;
  autoDriveStatus = false;
  private isFullscreen = false;

  constructor(public directionService: DirectionService, @Inject(DOCUMENT) private document: any) {
  }

  ngOnInit() {
    this.docElem = document.documentElement;
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

  get recStatusText(): string {
    return this.recStatus ? 'REC' : 'Waiting'
  }

  get recBtnText(): string {
    return this.recStatus ? 'Stop Rec' : 'Start Rec';
  }

  get autoDriveStatusText(): string {
    return this.autoDriveStatus ? 'AI' : 'Manual';
  }

  get autoDriveBtnText(): string {
    return this.autoDriveStatus ? 'Stop AutoDrive' : 'Start AutoDrive';
  }

  onClickRecBtn(): void {
    this.recStatus = !this.recStatus;
    this.directionService.sendRecCmd(this.recStatus);
  }

  onClickAutoDriveBtn(): void {
    this.autoDriveStatus = !this.autoDriveStatus;
    this.directionService.sendAutoDriveCmd(this.autoDriveStatus);
  }

  onClickScreenBtn(): void {
    this.isFullscreen = !this.isFullscreen;
    return this.isFullscreen ? this.openFullscreen() : this.closeFullscreen();
  }

  private openFullscreen(): void {
    if (this.docElem.requestFullscreen) {
      this.docElem.requestFullscreen();
    } else if (this.docElem.mozRequestFullScreen) {
      /* Firefox */
      this.docElem.mozRequestFullScreen();
    } else if (this.docElem.webkitRequestFullscreen) {
      /* Chrome, Safari and Opera */
      this.docElem.webkitRequestFullscreen();
    } else if (this.docElem.msRequestFullscreen) {
      /* IE/Edge */
      this.docElem.msRequestFullscreen();
    }
  }

  /* Close fullscreen */
  private closeFullscreen(): void {
    if (this.document.exitFullscreen) {
      this.document.exitFullscreen();
    } else if (this.document.mozCancelFullScreen) {
      /* Firefox */
      this.document.mozCancelFullScreen();
    } else if (this.document.webkitExitFullscreen) {
      /* Chrome, Safari and Opera */
      this.document.webkitExitFullscreen();
    } else if (this.document.msExitFullscreen) {
      /* IE/Edge */
      this.document.msExitFullscreen();
    }
  }

}
