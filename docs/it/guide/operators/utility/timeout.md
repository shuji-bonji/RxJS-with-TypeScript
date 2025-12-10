---
description: timeout è un operatore utility RxJS che lancia un errore se nessun valore viene emesso dall'Observable entro un tempo specificato. Ideale per elaborazione reattiva con vincoli temporali come controllo timeout richieste API, attesa risposte azioni utente o rilevamento ritardo stream. Può essere combinato con catchError per implementare comportamento di fallback, e l'inferenza dei tipi TypeScript permette elaborazione timeout type-safe.
---

# timeout - Configurazione Timeout

L'operatore `timeout` è un operatore che **lancia un errore se nessun valore viene emesso dall'Observable entro un tempo specificato**.
È spesso usato per elaborazione reattiva, come attendere una risposta a una richiesta API o un'operazione utente.


## 🔰 Sintassi e Operazione Base

Se il timeout non viene superato, l'operazione continua normalmente; se supera un certo periodo, si verifica un errore.

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('risposta')
  .pipe(
    delay(500), // 👈 Se impostato a 1500, output `Errore timeout: fallback`
    timeout(1000),
    catchError((err) => of('Errore timeout: fallback', err))
  )
  .subscribe(console.log);
// Output:
// risposta
```

In questo esempio, `'risposta'` viene normalmente visualizzato poiché il valore viene emesso dopo 500ms a causa di `delay(500)` e la condizione di `timeout(1000)` è soddisfatta.

Se viene specificato `delay(1200)`, viene emesso un `errore timeout` come segue:
```sh
Errore timeout: fallback
TimeoutErrorImpl {stack: 'Error\n    at _super (http://localhost:5174/node_mo…s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {…}}
```

[🌐 Documentazione Ufficiale RxJS - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 Esempio di Utilizzo Tipico

L'esempio seguente mostra sia un **pattern che causa un timeout se lo stream ritarda e non emette un valore** sia un **pattern che emette normalmente**.

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout avvenuto'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout scattato'))
  )
  .subscribe(console.log);
// Output:
// 0
// 1
// fallback: timeout scattato
// 2
```


## 🧪 Esempio di Codice Pratico (con UI)

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// Area di visualizzazione output
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>Esempio timeout:</h3>';
document.body.appendChild(timeoutOutput);

// Caso timeout riuscito
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>Stream Normale (Nessun Timeout):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Errore: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('Valore di fallback dopo errore');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valore: ${val}`;
    timeoutSuccess.appendChild(item);
  });

// Caso errore timeout
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>Stream Lento (Timeout Avviene):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Errore: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('Valore di fallback dopo timeout');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valore: ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ Riepilogo

- `timeout` è un operatore di controllo che **lancia un errore se nessuna emissione avviene entro un certo tempo**
- Efficace per elaborazione timeout durante attesa API di rete o operazioni UI
- Può essere combinato con `catchError` per specificare **comportamento di fallback**
