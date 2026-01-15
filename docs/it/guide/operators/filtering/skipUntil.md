---
description: L'operatore skipUntil salta tutti i valori dall'Observable originale fino a quando un altro Observable emette un valore, poi emette i valori normalmente. È utile per avvio ritardato basato sul tempo o elaborazione dopo che si verifica un evento specifico.
---

# skipUntil - Salta Fino a Trigger

L'operatore `skipUntil` **salta tutti i valori dall'Observable sorgente** fino a quando un Observable specificato (trigger di notifica) emette il suo primo valore. Dopo che il trigger di notifica emette, i valori successivi vengono emessi normalmente.


## 🔰 Sintassi e Utilizzo Base

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // Emetti valore ogni 0.5 secondi
const notifier$ = timer(2000); // Emetti valore dopo 2 secondi

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Output: 4, 5, 6, 7, 8, ...
// (I primi 2 secondi di valori 0, 1, 2, 3 vengono saltati)
```

**Flusso di operazione**:
1. `source$` emette 0, 1, 2, 3 → tutti saltati
2. Dopo 2 secondi, `notifier$` emette un valore
3. I valori successivi di `source$` (4, 5, 6, ...) vengono emessi normalmente

[🌐 Documentazione Ufficiale RxJS - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)


## 🆚 Confronto con takeUntil

`skipUntil` e `takeUntil` hanno comportamento opposto.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // Emetti valore ogni 0.5 secondi
const notifier$ = timer(2000); // Emetti valore dopo 2 secondi

// takeUntil: Prendi valori fino alla notifica
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Output: 0, 1, 2, 3 (si ferma dopo 2 secondi)

// skipUntil: Salta valori fino alla notifica
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Output: 4, 5, 6, 7, ... (inizia dopo 2 secondi)
```

| Operatore | Comportamento | Timing Completamento |
|---|---|---|
| `takeUntil(notifier$)` | **Prendi** valori fino alla notifica | Completa automaticamente quando notificato |
| `skipUntil(notifier$)` | **Salta** valori fino alla notifica | Quando lo stream sorgente completa |


## 💡 Pattern di Utilizzo Tipici

1. **Inizia Elaborazione Dati Dopo Autenticazione Utente**
   ```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // Salta dati fino a quando l'autenticazione completa
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`Elaborazione dati: ${data}`);
   });

   // Autenticazione completa dopo 3 secondi
   setTimeout(() => {
     console.log('Autenticazione completata!');
     authenticated$.next();
   }, 3000);
   // Dopo 3 secondi, emette "Elaborazione dati: 3", "Elaborazione dati: 4", ...
   ```

2. **Inizia Elaborazione Eventi Dopo Completamento Caricamento Iniziale**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'Click';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // Ignora click fino a quando l'app è pronta
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('Click elaborato');
   });

   // App pronta dopo 2 secondi
   setTimeout(() => {
     console.log('App pronta');
     appReady$.next(true);
   }, 2000);
   ```

3. **Avvio Ritardato Basato su Timer**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { skipUntil, scan } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Conta';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');
   const startTime$ = timer(3000); // Dopo 3 secondi

   // Non contare click fino a quando passano 3 secondi
   clicks$.pipe(
     skipUntil(startTime$),
     scan(count => count + 1, 0)
   ).subscribe(count => {
     console.log(`Conteggio: ${count}`);
   });

   console.log('Il conteggio inizia dopo 3 secondi...');
   ```


## 🧠 Esempio di Codice Pratico (Conto alla Rovescia Gioco)

Esempio di ignorare i click durante il conto alla rovescia prima dell'inizio del gioco e abilitare i click dopo che il conto alla rovescia termina.

```ts
import { fromEvent, timer, interval } from 'rxjs';
import { skipUntil, take, scan } from 'rxjs';

// Crea elementi UI
const container = document.createElement('div');
document.body.appendChild(container);

const countdown = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'Conto alla rovescia...';
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = 'Click!';
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'Punteggio: 0';
container.appendChild(scoreDisplay);

// Conto alla rovescia (3 secondi)
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `Inizia tra ${3 - n} secondi...`;
  },
  complete: () => {
    countdown.textContent = 'Inizio Gioco!';
    button.disabled = false;
  }
});

// Notifica inizio gioco
const gameStart$ = timer(3000);

// Eventi click (salta fino all'inizio del gioco)
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),
  scan(score => score + 10, 0)
).subscribe(score => {
  scoreDisplay.textContent = `Punteggio: ${score}`;
});
```

In questo codice, i click vengono ignorati durante il conto alla rovescia di 3 secondi, e solo i click dopo che il conto alla rovescia termina vengono riflessi nel punteggio.


## 🎯 Differenza Tra skip e skipUntil

```ts
import { interval, timer } from 'rxjs';
import { skip, skipUntil } from 'rxjs';

const source$ = interval(500);

// skip: Salta primi N valori per conteggio
source$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, ...

// skipUntil: Salta fino a quando un altro Observable si attiva
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);
// Output: 3, 4, 5, 6, ... (stesso risultato, ma metodo di controllo diverso)
```

| Operatore | Condizione Skip | Caso d'Uso |
|---|---|---|
| `skip(n)` | Salta primi n per conteggio | Skip conteggio fisso |
| `skipWhile(predicate)` | Salta mentre la condizione è soddisfatta | Skip basato su condizione |
| `skipUntil(notifier$)` | Salta fino a quando un altro Observable si attiva | Skip basato su evento/tempo |


## 📋 Utilizzo Type-Safe

Esempio di implementazione type-safe utilizzando i generics di TypeScript.

```ts
import { Observable, Subject, fromEvent } from 'rxjs';
import { skipUntil, map } from 'rxjs';

interface GameState {
  status: 'waiting' | 'ready' | 'playing' | 'finished';
}

interface ClickEvent {
  timestamp: number;
  x: number;
  y: number;
}

class Game {
  private gameReady$ = new Subject<void>();
  private state: GameState = { status: 'waiting' };

  startGame(element: HTMLElement): Observable<ClickEvent> {
    const clicks$ = fromEvent<MouseEvent>(element, 'click').pipe(
      map(event => ({
        timestamp: Date.now(),
        x: event.clientX,
        y: event.clientY
      } as ClickEvent)),
      skipUntil(this.gameReady$)
    );

    // Notifica pronto
    setTimeout(() => {
      this.state = { status: 'ready' };
      this.gameReady$.next();
      console.log('Gioco pronto!');
    }, 2000);

    return clicks$;
  }
}

// Esempio di utilizzo
const game = new Game();
const canvas = document.createElement('div');
canvas.style.width = '300px';
canvas.style.height = '200px';
canvas.style.border = '1px solid black';
canvas.textContent = 'Clicca qui';
document.body.appendChild(canvas);

game.startGame(canvas).subscribe(click => {
  console.log(`Posizione click: (${click.x}, ${click.y})`);
});
```


## 🔄 Combinare skipUntil e takeUntil

Per prendere valori solo per un periodo specifico, combina entrambi.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500);
const start$ = timer(2000); // Inizia dopo 2 secondi
const stop$ = timer(5000);  // Ferma dopo 5 secondi

source$.pipe(
  skipUntil(start$), // Salta fino a 2 secondi
  takeUntil(stop$)   // Ferma a 5 secondi
).subscribe({
  next: console.log,
  complete: () => console.log('Completo')
});
// Output: 4, 5, 6, 7, 8, 9, Completo
// (Solo valori tra 2-5 secondi)
```

**Timeline**:
```
0s    1s    2s    3s    4s    5s
|-----|-----|-----|-----|-----|
0  1  2  3  4  5  6  7  8  9  10
      ↑           ↑
   skip inizio  take fine
   (da 4)    (a 9)
```


## ⚠️ Errori Comuni

> [!IMPORTANT]
> `skipUntil` è valido solo per la **prima emissione** dell'Observable di notifica. La seconda e le emissioni successive vengono ignorate.

### Sbagliato: Observable di Notifica Si Attiva Più Volte

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ❌ Esempio sbagliato: Chiamare next più volte ha effetto solo una volta
setTimeout(() => notifier$.next(), 1000);
setTimeout(() => notifier$.next(), 2000); // Questo non ha significato
```

### Corretto: Comprendi che Solo la Prima Emissione è Valida

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ✅ Esempio corretto: Chiama next solo una volta
setTimeout(() => {
  console.log('Skip terminato');
  notifier$.next();
  notifier$.complete(); // Best practice completare
}, 1000);
```


## 🎓 Riepilogo

### Quando Usare skipUntil
- ✅ Quando vuoi iniziare l'elaborazione dopo che si verifica un evento specifico
- ✅ Quando vuoi abilitare operazioni utente dopo che l'inizializzazione completa
- ✅ Quando serve avvio ritardato basato sul tempo
- ✅ Quando vuoi iniziare l'elaborazione dati dopo che l'autenticazione completa

### Combinazione con takeUntil
- ✅ Quando vuoi prendere valori solo per un periodo specifico (skipUntil + takeUntil)

### Note
- ⚠️ Solo la prima emissione dell'Observable di notifica è valida
- ⚠️ Se l'Observable di notifica non emette, tutti i valori continuano ad essere saltati
- ⚠️ La subscription viene mantenuta fino al completamento dello stream sorgente


## 🚀 Prossimi Passi

- **[skip](/it/guide/operators/filtering/skip)** - Impara come saltare i primi N valori
- **[take](/it/guide/operators/filtering/take)** - Impara come prendere i primi N valori
- **[takeUntil](../utility/takeUntil)** - Impara come prendere valori fino a quando un altro Observable si attiva
- **[filter](/it/guide/operators/filtering/filter)** - Impara come filtrare in base a condizioni
- **[Esempi Pratici Operatori di Filtraggio](/it/guide/operators/filtering/practical-use-cases)** - Impara casi d'uso reali
