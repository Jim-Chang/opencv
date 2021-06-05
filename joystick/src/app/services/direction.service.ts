import { Injectable } from '@angular/core';
import { HttpClient, HttpParams } from '@angular/common/http';
import { Direction, DirectionResponse } from 'Types/direction.type';
import { Observable, Subject, of } from 'rxjs';
import { map, switchMap } from 'rxjs/operators';
import { JoystickEvent } from 'ngx-joystick';

@Injectable({
  providedIn: 'root'
})
export class DirectionService {

  private directionApi = '/api/direction';
  private eventSubject = new Subject<JoystickEvent>();

  constructor(private http: HttpClient) {
    this.eventSubject.pipe(
      map(event => {
        return {
          speed: Math.round(event.data.distance),
          diff: Math.round(event.data.instance.frontPosition.x),
        }
      }),
      switchMap(direction => this.sendCmd(direction))
    ).subscribe(res => {
      console.log(res);
    });
  }

  receiveJoystickEvent(event: JoystickEvent): void {
    this.eventSubject.next(event);
  }

  sendCmd(direction: Direction): Observable<DirectionResponse> {
    console.log(direction);
    return of({ result: 'ok' });
    // return this.http.post<DirectionResponse>(this.directionApi, direction);
  }
}
