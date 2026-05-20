---
description: "Conoscere le caratteristiche, l'implementazione e l'uso dei principali scheduler di RxJS, come asyncScheduler e queueScheduler. Comprendete le differenze tra macro task, micro task ed elaborazione sincrona e imparate i tempi di esecuzione e le caratteristiche di ogni scheduler. Utilizzandoli in modo appropriato, è possibile ottimizzare le prestazioni e il comportamento dell'applicazione."
---

# Tipi di scheduler e come utilizzarli

RxJS fornisce diversi scheduler per diverse applicazioni. Ogni scheduler ha i suoi tempi di esecuzione e le sue caratteristiche specifiche e può essere utilizzato in modo appropriato per ottimizzare le prestazioni e il comportamento dell'applicazione.

## Classificazione degli scheduler

Gli scheduler di RxJS rientrano in tre categorie principali.

1. **Macro task**: eseguiti nella coda di task successiva nel ciclo degli eventi
2. **Micro-task**: eseguiti immediatamente dopo il completamento del task corrente e prima dell'inizio del task successivo
3.**Elaborazione sincrona**: esecuzione immediata

Per ulteriori informazioni, vedere [Fondamenti di task e scheduler](./task-and-scheduler-basics.md).

## Schedulatori principali.

### asyncScheduler

#### Caratteristiche.
- **Implementazione interna**: usa setTimeout.
- **Tempistiche di esecuzione**: macro attività
- **Uso**: elaborazione asincrona generale, elaborazione time-lapse.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Avvio');

of('Elaborazione asincrona')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Uscita:
// 1: Avvio
// 2: Fine
// 3: Elaborazione asincrona
```

#### Caso d'uso

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simulazione di un calcolo pesante
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Risultati del calcolo: ${result}`);
  });
```

### Scheduler delle code

#### Caratteristiche.
- **Implementazione interna**: micro coda di task
- **Tempistiche di esecuzione**: all'interno del task corrente (sembra sincrono)
- **Usi**: accodamento di task, ottimizzazione della ricorsione

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Avvio');

of('Elaborazione in coda')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Fine');

// Uscita:
// 1: Avvio
// 2: Elaborazione in coda
// 3: Fine
```

#### Caso d'uso

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Ottimizzazione dell'elaborazione ricorsiva
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Caratteristiche.
- **Implementazione interna**: Promise.resolve().then() o setImmediate
- **Tempistiche di esecuzione**: microtasks
- **Utilizzo**: per l'esecuzione asincrona il più rapidamente possibile

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Avvio');

of('ASAPElaborazione')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Uscita:
// 1: Avvio
// 2: Fine
// 3: ASAPElaborazione
```

#### Caso d'uso

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Ottimizzazione degli eventi di movimento del mouse
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UIElaborazione degli aggiornamenti di
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Caratteristiche.
- **Implementazione interna**: requestAnimationFrame
- **Tempistiche di esecuzione**: prima del successivo rendering dello schermo
- **Utilizzo**: Animazione, processo di disegno per 60fps.

#### Esempio di una semplice animazione di rotazione

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTMLCreazione di elementi
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Impostazione delle animazioni
let rotation = 0;

// 60fpsNei2Secondi di animazione
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2Secondi = 120Fotogramma
    map(() => {
      rotation += 3;  // 1Ogni fotogramma3Grado di rotazione
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOMRotazione effettiva dell'elemento
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Perché abbiamo bisogno di Scheduler?

Lo Scheduler dell'animazione esegue le sue operazioni in modo sincrono con il ciclo di disegno del browser, con i seguenti vantaggi.

1.**Animazioni fluide**: poiché l'elaborazione è sincronizzata con i tempi di rendering del browser (in genere 60 fps), è possibile ottenere animazioni fluide e senza interruzioni.
2.**Uso efficiente delle risorse**: quando il browser disattiva la scheda, l'esecuzione di requestAnimationFrame viene automaticamente messa in pausa, evitando un uso non necessario della CPU.
3, **Prevenzione dello sfarfallio dello schermo**: lo sfarfallio dello schermo e i fotogrammi incompleti vengono evitati perché il calcolo viene completato in modo affidabile prima che lo schermo venga disegnato.

Ecco un confronto tra `setInterval` e `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalAnimazione inefficiente che utilizza
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // approssimativamente60fps

// Problemi:
// - Non sincronizzato con i tempi di rendering del browser
// - Continua ad essere eseguita anche nelle schede in background
// - Accurato60fpsnon può essere garantita

// ✅ animationFrameSchedulerAnimazione efficiente con l'utilizzo di
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Vantaggi
// - Sincronizzato con i tempi di disegno del browser
// - Pausa automatica nelle schede in background
// - Stabile60fpsConsente
```

#### Esempio di animazione che segue il mouse

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Creazione di un cerchio da seguire
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Trasparente agli eventi del mouse
document.body.appendChild(circle);

// Posizione corrente e di destinazione
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Monitoraggio degli eventi di movimento del mouse
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Ciclo di animazione
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Impostare la posizione del mouse come target
    targetX = x;
    targetY = y;
    
    // Movimento graduale dalla posizione corrente verso la posizione di destinazione々Movimento graduale (easing) dalla posizione corrente alla posizione di destinazione
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOMAggiornamento dell'elemento
    circle.style.left = `${currentX - 15}px`;  // Adattamento alla posizione centrale
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guida all'utilizzo di diversi scheduler

### Confronto per tempi di esecuzione

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Avvio');

of('Elaborazione asincrona')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Uscita:
// 1: Avvio
// 2: Fine
// 3: Elaborazione asincrona
```

### Criteri di selezione per applicazione


## Casi d'uso pratici

### Elaborazione di grandi quantità di dati


## Elaborazione di messaggi WebSocket

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Nota: Questo è pseudo codice per illustrare il concetto
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Tratta come una stringa
});

socket$
  .pipe(
    // Elaborazione di messaggi che richiedono una risposta rapida
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Messaggio ricevuto:', msg);
}
```

## Controllo degli errori

Utilizzando lo scheduler con l'operatore retry, è possibile controllare con precisione la tempistica dei tentativi.

#### Controllo di base dei tentativi

L'opzione `delay` dell'operatore `retry` utilizza internamente lo Scheduler per controllare l'intervallo di retry.


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Avvio');

of('Elaborazione asincrona')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Uscita:
// 1: Avvio
// 2: Fine
// 3: Elaborazione asincrona
```

#### Utilizzo dello scheduler con back-off esponenziale

Come controllo più avanzato, il backoff esponenziale può essere implementato combinando `retryWhen` e `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    // RxJS 7.3+ 推奨: retry({ count, delay }) 形式
    retry({
      count: 3, // Max.3回まで再試行
      delay: (error, retryCount) => {
        // 指数バックオフ: 1秒, 2秒, 4秒...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 リトライ ${retryCount}回目 (${delayTime}ms後)`);

        // timer は内部的に asyncScheduler を使用
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ 成功:', result),
    error: error => {
      console.log('❌ Max.リトライ数に到達');
      console.log('❌ 最終Errore:', error.message);
    }
  });

// OutputEs.:
// 🔄 リトライ 1回目 (1000ms後)
// 🔄 リトライ 2回目 (2000ms後)
// 🔄 リトライ 3回目 (4000ms後)
// ❌ Max.リトライ数に到達
// ❌ 最終Errore: Temporary error
```

#### Quando si specifica esplicitamente uno Scheduler

Specificare esplicitamente uno scheduler specifico consente un controllo più flessibile, come la sostituzione con Scheduler durante i test.


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Avvio');

of('Elaborazione asincrona')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Uscita:
// 1: Avvio
// 2: Fine
// 3: Elaborazione asincrona
```


> - Modelli di implementazione dettagliati e metodi di debug per la gestione dei retry sono descritti nella pagina [retry e catchError](/it/guide/error-handling/retry-catch).
> Uso dettagliato dell'operatore retry.
> Modelli di combinazione con catchError.
> Tecniche di debug per i tentativi (ad esempio, monitoraggio del numero di tentativi, registrazione, ecc.)

## Impatto sulle prestazioni.

### Sovraccarico dello schedulatore


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Avvio');

of('Elaborazione asincrona')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Uscita:
// 1: Avvio
// 2: Fine
// 3: Elaborazione asincrona
```

## Riepilogo.

La scelta dello scheduler ha un impatto significativo sulle prestazioni e sulla reattività di un'applicazione. La comprensione delle caratteristiche di ogni scheduler e il loro utilizzo nelle giuste situazioni garantiranno un funzionamento efficiente e senza intoppi. Come linea guida generale,

- Scheduler` per l'elaborazione asincrona generale.
- Scheduler` per l'elaborazione ricorsiva e l'accodamento sincrono.
- Scheduler` per tempi di risposta rapidi
- Scheduler per le animazioni

per l'animazione.