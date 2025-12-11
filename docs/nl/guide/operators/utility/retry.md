---
description: De retry operator herabonneert en probeert de bron een gespecificeerd aantal keren opnieuw wanneer er een fout optreedt in Observable. Dit is nuttig voor herstel van tijdelijke communicatiefouten, zoals netwerkfouten, of voor processen die kunnen slagen als ze opnieuw worden geprobeerd na een fout.
---

# retry - Opnieuw proberen bij fout

De `retry` operator is een operator die **de bron Observable een gespecificeerd aantal keren herabonneert** wanneer er een fout optreedt.
Het is geschikt voor **processen die kunnen slagen als ze opnieuw worden geprobeerd na fout**, zoals tijdelijke netwerkfouten.

## 🔰 Basissyntax en werking

### retry(count) - Basisvorm

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('Tijdelijke fout'))
  .pipe(
    retry(2), // Probeer tot 2 keer opnieuw
    catchError((error) => of(`Definitieve fout: ${error.message}`))
  )
  .subscribe(console.log);
// Uitvoer:
// Definitieve fout: Tijdelijke fout
```

In dit voorbeeld worden tot twee keer opnieuw geprobeerd na de eerste fout, en wordt een bericht uitgevoerd bij fallback als alles faalt.

### retry(config) - Configuratieobject formaat (RxJS 7.4+)

In RxJS 7.4 en later is meer gedetailleerde controle mogelijk door een configuratieobject door te geven.

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

let attemptCount = 0;

throwError(() => new Error('Tijdelijke fout'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Poging ${attemptCount}`);
      }
    }),
    retry({
      count: 2,           // Probeer tot 2 keer opnieuw
      delay: 1000,        // Wacht 1 seconde voor opnieuw proberen (gebruikt asyncScheduler intern)
      resetOnSuccess: true // Reset telling bij succes
    }),
    catchError((error) => of(`Definitieve fout: ${error.message}`))
  )
  .subscribe(console.log);

// Uitvoer:
// Poging 1
// Poging 2
// Poging 3
// Definitieve fout: Tijdelijke fout
```

> [!NOTE] Timing controle voor opnieuw proberen
> Wanneer de `delay` optie is gespecificeerd, wordt **asyncScheduler** intern gebruikt. Voor meer gedetailleerde timing controle bij opnieuw proberen (exponentiële backoff, etc.), zie [Scheduler types en gebruik - Fout retry controle](/nl/guide/schedulers/types#error-retry-control).

[🌐 RxJS Officiële Documentatie - retry](https://rxjs.dev/api/index/function/retry)

## 💡 Typisch gebruiksvoorbeeld

Het volgende voorbeeld is een configuratie die **asynchrone verwerking met willekeurig succes/falen** tot 3 keer opnieuw probeert.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`Fout #${attempt}`));
      } else {
        return of(`Succes #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`Definitieve fout: ${err.message}`))
  )
  .subscribe(console.log);
// Uitvoer:
// Succes #1
// Succes #5
// Succes #6
// Definitieve fout: Fout #7
```

## 🧪 Praktisch codevoorbeeld (met UI)

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// Uitvoerweergavegebied
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>retry voorbeeld (API-verzoek simulatie):</h3>';
document.body.appendChild(retryOutput);

// Verzoekstatus weergave
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// API-verzoek dat willekeurig slaagt of faalt
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `Poging #${attemptCount} Verzoek verzenden...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `Poging #${attemptCount} Mislukt: Netwerkfout`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('Netwerkfout'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `Poging #${attemptCount} Succes!`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'Data succesvol opgehaald' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `Alle pogingen mislukt: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: 'Opnieuw proberen mislukt' });
    })
  );
}

// Verzoek startknop
const startButton = document.createElement('button');
startButton.textContent = 'Start verzoek';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `Eindresultaat: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `Eindresultaat: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ Samenvatting

- `retry(n)` probeert tot `n` keer opnieuw als Observable faalt
- `retry` wordt **opnieuw geprobeerd totdat het succesvol voltooid** (aanhoudende fout resulteert in een fout)
- Nuttig voor **asynchrone API's en netwerkverzoeken** waar tijdelijke fouten optreden
- Vaak gecombineerd met `catchError` om **fallback-verwerking** te specificeren
- Vanaf RxJS 7.4+ is het mogelijk om `delay`, `resetOnSuccess`, etc. te specificeren in configuratieobject formaat

## Gerelateerde pagina's

- [retry en catchError](/nl/guide/error-handling/retry-catch) - Patronen voor het combineren van retry en catchError, praktische gebruiksvoorbeelden
- [Retry debuggen](/nl/guide/error-handling/retry-catch#retry-debugging) - Hoe pogingstelling te volgen (5 implementatiepatronen)
- [Scheduler types en gebruik](/nl/guide/schedulers/types#error-retry-control) - Gedetailleerde retry timing controle, exponentiële backoff implementatie
- [RxJS debugtechnieken](/nl/guide/debugging/#scenario-6-track-retry-attempt-count) - Retry debug scenario's
