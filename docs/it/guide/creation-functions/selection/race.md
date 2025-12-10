---
description: La Funzione di Creazione race realizza un processo di concatenazione speciale che adotta solo il primo stream che emette un valore tra più Observable e ignora gli altri successivamente.
titleTemplate: ':title | RxJS'
---

# race - adotta lo stream che ha emesso il valore per primo

`race` è una Funzione di Creazione di concatenazione speciale che sfrutta **solo il primo Observable che emette un valore** tra più Observable,
e ignora gli altri Observable.


## Sintassi e utilizzo base

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lento (5 secondi)'));
const fast$ = timer(2000).pipe(map(() => 'Veloce (2 secondi)'));

race(slow$, fast$).subscribe(console.log);
// Output: Veloce (2 secondi)
```

- Solo l'Observable che ha emesso il valore per primo è il vincitore e continua con gli stream successivi.
- Gli altri Observable vengono ignorati.

[🌐 Documentazione Ufficiale RxJS - `race`](https://rxjs.dev/api/index/function/race)


## Pattern di utilizzo tipici

- **Elabora la prima tra più azioni utente (click, pressioni tasti, scrolling)**
- **Adotta il primo tra più trigger, come invio manuale e salvataggio automatico**
- **Visualizza i primi dati completati tra più processi di acquisizione dati**

## Esempi di codice pratici (con UI)

Simula una gara per adottare solo il primo emesso da tre stream che sparano a tempi diversi.

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Crea area output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico race:</h3>';
document.body.appendChild(output);

// Observable con timing diversi
const slow$ = timer(5000).pipe(map(() => 'Lento (dopo 5 secondi)'));
const medium$ = timer(3000).pipe(map(() => 'Medio (dopo 3 secondi)'));
const fast$ = timer(2000).pipe(map(() => 'Veloce (dopo 2 secondi)'));

const startTime = Date.now();

// Messaggio di inizio gara
const waiting = document.createElement('div');
waiting.textContent = 'Gara iniziata... In attesa del primo stream da emettere.';
output.appendChild(waiting);

// Esegui race
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>Vincitore:</strong> ${winner} (Tempo trascorso: ${elapsed} secondi)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ Solo il primo Observable che ha emesso un valore viene selezionato.';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- Dopo 2 secondi, il primo `fast$` viene emesso e successivamente solo `fast$` viene emesso.
- Le altre emissioni di `medium$` e `slow$` verranno ignorate.


## Operatori Correlati

- **[raceWith](/it/guide/operators/combination/raceWith)** - Versione Pipeable Operator (usato nella pipeline)
- **[timeout](/it/guide/operators/utility/timeout)** - Operatore solo timeout
- **[merge](/it/guide/creation-functions/combination/merge)** - Funzione di Creazione che unisce tutti gli stream
