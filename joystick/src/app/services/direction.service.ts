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

  private directionApi = '/api/direction';
  private eventSubject = new Subject<JoystickEvent>();

  constructor(private http: HttpClient) {
    this.eventSubject.pipe(
      debounceTime(15),
      map(event => {
        const y = -event.data.instance.frontPosition.y;
        const sign = y / Math.abs(y);
        return {
          speed: Math.round(event.data.distance) * sign | 0,
          diff: Math.round(event.data.instance.frontPosition.x) | 0,
        }
      }),
      switchMap(direction => this.sendCmd(direction))
    ).subscribe(res => {
      console.log(res);
    });
  }

  receiveMoveEvent(event: JoystickEvent): void {
    this.eventSubject.next(event);
  }

  receiveStopEvent(): void {
    const direction = {
      speed: 0,
      diff: 0,
    };
    this.sendCmd(direction).subscribe(res => {
      console.log(res);
    });
  }

  sendCmd(direction: Direction): Observable<DirectionResponse> {
    console.log(direction);
    // return of({ result: 'ok' });
    return this.http.post<DirectionResponse>(this.directionApi, direction);
  }
}