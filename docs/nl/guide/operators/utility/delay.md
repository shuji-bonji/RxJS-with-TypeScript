---
description: De delay operator vertraagt de uitgiftetiming van elke waarde in de Observable met een gespecificeerde hoeveelheid tijd, wat effectief is voor UI-regie en asynchrone controle.
---

# delay - Waardevertraging

De `delay` operator wordt gebruikt om de uitgifte van elke waarde in een stream met een gespecificeerde hoeveelheid tijd te vertragen.
Dit is nuttig voor het faseren van animaties en het aanpassen van de timing van feedbackweergave aan de gebruiker.


## 🔰 Basissyntax en werking

Dit is de minimale configuratie om een waarde na een bepaalde tijd uit te geven.

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Hallo')
  .pipe(
    delay(1000) // Geef waarde uit na 1 seconde
  )
  .subscribe(console.log);
// Uitvoer:
// Hallo
```

In dit voorbeeld wordt de waarde gemaakt door `of('Hallo')` ontvangen door `subscribe()` met een vertraging van 1 seconde.

[🌐 RxJS Officiële Documentatie - delay](https://rxjs.dev/api/index/function/delay)

## 💡 Typisch gebruiksvoorbeeld

Dit is een voorbeeld van het gebruik van delay om de timing van uitgifte aan te passen in een situatie waar meerdere waarden worden uitgegeven.

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // A onmiddellijk, B na 1 seconde, C na 2 seconden
    )
  )
  .subscribe(console.log);
// Uitvoer:
// A
// B
```

Op deze manier is het ook mogelijk om een aparte vertraging voor elke waarde in te stellen door het te combineren met `concatMap`.


## 🧪 Praktisch codevoorbeeld (met UI)

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// Uitvoerweergavegebied
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>delay voorbeeld:</h3>';
document.body.appendChild(delayOutput);

// Functie om huidige tijd weer te geven
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('nl-NL', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// Registreer starttijd
addTimeLog('Start');

// Waardenreeks
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`Voordat waarde ${val} wordt uitgegeven`)),
    delay(1000), // 1 seconde vertraging
    tap((val) => addTimeLog(`Waarde ${val} uitgegeven na 1 seconde`))
  )
  .subscribe();
```


## ✅ Samenvatting

- `delay` is een operator voor **het controleren van de timing van Observable-uitvoer**
- Kan worden gecombineerd met `concatMap` om **vertraging per waarde te controleren**
- Nuttig voor **asynchrone aanpassingen** om UX te verbeteren, zoals uitvoer naar UI en timer-regie
