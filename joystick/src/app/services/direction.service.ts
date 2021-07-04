import { Injectable } from '@angular/core';
import { HttpClient, HttpParams } from '@angular/common/http';
import { Direction, DirectionResponse } from 'Types/direction.type';
import { JoystickEvent } from 'ngx-joystick';

@Injectable({
  providedIn: 'root'
})
export class DirectionService {

  private recApi = '/api/rec';
  private autoDriveApi = '/api/auto_drive';

  private worker: Worker;
  private SPEED_MIN = 50;
  private SPEED_MAX = 100;
  private SPEED_STEP = 5;
  private STEER_MIN = 0;
  private STEER_MAX = 50;
  private STEER_STEP = 5;

  lastDirection: Direction = {
    speed: 0,
    diff: 0,
  };
  isWorkerReady = false;

  constructor(private http: HttpClient) {
    if (typeof Worker !== 'undefined') {
      this.worker = new Worker(new URL('./direction.worker', import.meta.url));
      this.isWorkerReady = true;
    } else {
      console.log('not support web worker.')
    }
  }

  private calculateValue(value: number, min: number, max: number, step: number): number {
    const sign = value / Math.abs(value);
    const scaleValue = Math.abs(value) / 100 * (max - min) + min;
    return Math.floor(scaleValue / step) * step * sign;
  }

  private getSign(value: number): number {
    return value / Math.abs(value);
  }

  receiveMotorMoveEvent(event: JoystickEvent): void {
    const sign = this.getSign(-event.data.instance.frontPosition.y);

    const speed = this.calculateValue(event.data.distance, this.SPEED_MIN, this.SPEED_MAX, this.SPEED_STEP) * sign | 0;
    if (speed != this.lastDirection.speed) {
      this.lastDirection.speed = speed;
      this.sendMoveCmdByWorker(this.lastDirection);
    }
  }

  receiveSteerMoveEvent(event: JoystickEvent): void {
    const sign = this.getSign(event.data.instance.frontPosition.x);

    const diff = this.calculateValue(event.data.distance, this.STEER_MIN, this.STEER_MAX, this.STEER_STEP) * sign | 0;
    if (diff != this.lastDirection.diff) {
      this.lastDirection.diff = diff;
      this.sendMoveCmdByWorker(this.lastDirection);
    }
  }

  receiveMotorEndEvent(): void {
    if (this.lastDirection.speed != 0) {
      this.lastDirection.speed = 0;
      this.sendMoveCmdByWorker(this.lastDirection);
    }
  }

  receiveSteerEndEvent(): void {
    if (this.lastDirection.diff != 0) {
      this.lastDirection.diff = 0;
      this.sendMoveCmdByWorker(this.lastDirection);
    }
  }

  sendRecCmd(isRec: boolean): void {
    console.log(`Send rec cmd: {isRec}`);
    const params = new HttpParams().append('status', isRec.toString());
    this.http.get<DirectionResponse>(this.recApi, { params }).subscribe((result) => console.log(result));
  }

  sendAutoDriveCmd(isEnable: boolean): void {
    console.log(`Send auto drive cmd: {isEnable}`);
    const params = new HttpParams().append('status', isEnable.toString());
    this.http.get<DirectionResponse>(this.autoDriveApi, { params }).subscribe((result) => console.log(result));
  }

  private sendMoveCmdByWorker(direction: Direction): void {
    this.worker.postMessage(direction);
  }
}
