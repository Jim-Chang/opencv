/// <reference lib="webworker" />
import { Direction } from 'Types/direction.type';
import { ajax } from 'rxjs/ajax';

const directionApi = '/api/direction';

addEventListener('message', ({ data }) => {
  // console.log('[app worker] event listener', data);
  sendMoveCmd(data);
});

function sendMoveCmd(direction: Direction): void {
  ajax({
    url: directionApi,
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    async: true,
    timeout: 2000,
    body: direction,
  }).subscribe(res => console.log(res));
}

