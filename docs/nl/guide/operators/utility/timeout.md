---
description: timeout is een RxJS utility operator die een fout gooit als er geen waarde wordt uitgezonden door Observable binnen een gespecificeerde tijd. Ideaal voor tijdgebonden reactieve verwerking zoals API-verzoek timeout-controle, wachten op gebruikersactie-responsen, of stream-vertragingsdetectie. Het kan worden gecombineerd met catchError om fallback-gedrag te implementeren, en TypeScript type-inferentie maakt type-veilige timeout-verwerking mogelijk.
---

# timeout - Timeout-configuratie

De `timeout` operator is een operator die **een fout gooit als er geen waarde wordt uitgezonden door Observable binnen een gespecificeerde tijd**.
Het wordt vaak gebruikt voor reactieve verwerking, zoals wachten op een respons op een API-verzoek of gebruikersoperatie.


## 🔰 Basissyntax en werking

Als de timeout niet wordt overschreden, gaat de operatie door zoals gebruikelijk; als het een bepaalde periode overschrijdt, treedt er een fout op.

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('respons')
  .pipe(
    delay(500), // 👈 Als ingesteld op 1500, geeft `Timeout-fout: fallback` uit
    timeout(1000),
    catchError((err) => of('Timeout-fout: fallback', err))
  )
  .subscribe(console.log);
// Uitvoer:
// respons
```

In dit voorbeeld wordt `'respons'` normaal weergegeven aangezien de waarde wordt uitgezonden na 500ms door `delay(500)` en de voorwaarde van `timeout(1000)` is voldaan.

Als `delay(1200)` wordt gespecificeerd, wordt een `timeout-fout` als volgt uitgevoerd:
```sh
Timeout-fout: fallback
TimeoutErrorImpl {stack: 'Error\n    at _super (http://localhost:5174/node_mo…s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {…}}
```

[🌐 RxJS Officiële Documentatie - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 Typisch gebruiksvoorbeeld

Het volgende voorbeeld toont zowel een **patroon dat een timeout veroorzaakt als de stream vertraagt en geen waarde uitzendt** als een **patroon dat normaal uitzendt**.

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout opgetreden'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout getriggerd'))
  )
  .subscribe(console.log);
// Uitvoer:
// 0
// 1
// fallback: timeout getriggerd
// 2
```


## 🧪 Praktisch codevoorbeeld (met UI)

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// Uitvoerweergavegebied
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>timeout voorbeeld:</h3>';
document.body.appendChild(timeoutOutput);

// Timeout succes geval
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>Normale stream (geen timeout):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Fout: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('Fallback-waarde na fout');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Waarde: ${val}`;
    timeoutSuccess.appendChild(item);
  });

// Timeout fout geval
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>Trage stream (timeout treedt op):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Fout: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('Fallback-waarde na timeout');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Waarde: ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ Samenvatting

- `timeout` is een controle-operator die **een fout gooit als er geen uitgifte optreedt binnen een bepaalde tijd**
- Effectief voor timeout-verwerking tijdens het wachten op netwerk-API's of UI-operaties
- Kan worden gecombineerd met `catchError` om **fallback-gedrag** te specificeren
