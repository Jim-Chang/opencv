import { Injectable } from '@angular/core';
import { HttpClient, HttpParams } from '@angular/common/http';
import { Direction, DirectionResponse } from 'Types/direction.type';
import { Observable, Subject, of } from 'rxjs';
import { map, switchMap, debounceTime } from 'rxjs/operators';
import { JoystickEvent } from 'ngx-joystick';

@Injectable({
  providedIn: 'root'
})
export class DirectionService {

  private recApi = '/api/rec';
  private directionApi = '/api/direction';
  private eventSubject = new Subject<Direction>();

  lastDirection: Direction = {
    speed: 0,
    diff: 0,
  };

  constructor(private http: HttpClient) {
    this.eventSubject.pipe(
      debounceTime(15),
      switchMap(direction => this.sendMoveCmd(direction))
    ).subscribe(res => {
      console.log(res);
    });
  }

  private roundValue(value: number): number {
    const sign = value / Math.abs(value);
    return Math.floor(Math.abs(value) / 10) * 10 * sign;
  }

  private getSign(value: number): number {
    return value / Math.abs(value);
  }

  receiveMotorMoveEvent(event: JoystickEvent): void {
    const sign = this.getSign(-event.data.instance.frontPosition.y);
    this.lastDirection.speed = this.roundValue(event.data.distance) * sign | 0;
    this.eventSubject.next(this.lastDirection);
  }

  receiveSteerMoveEvent(event: JoystickEvent): void {
    const sign = this.getSign(event.data.instance.frontPosition.x);
    this.lastDirection.diff = this.roundValue(event.data.distance) * sign / 2 | 0;
    this.eventSubject.next(this.lastDirection);
  }

  receiveMotorEndEvent(): void {
    this.lastDirection.speed = 0
    this.eventSubject.next(this.lastDirection);
  }

  receiveSteerEndEvent(): void {
    this.lastDirection.diff = 0
    this.eventSubject.next(this.lastDirection);
  }

  sendMoveCmd(direction: Direction): Observable<DirectionResponse> {
    console.log(direction);
    // return of({ result: 'ok' });
    return this.http.post<DirectionResponse>(this.directionApi, direction);
  }

  sendRecCmd(isRec: boolean): void {
    console.log(`Send rec cmd: {isRec}`);
    const params = new HttpParams().append('status', isRec.toString());
    this.http.get<DirectionResponse>(this.recApi, { params }).subscribe((result) => console.log(result));
  }
}
