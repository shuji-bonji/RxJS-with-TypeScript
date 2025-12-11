---
description: dematerialize is een RxJS utility operator die Notification-objecten herstelt naar normale notificaties (next, error, complete) en de omgekeerde transformatie van materialize uitvoert. Het is ideaal voor het herstellen van notificaties na verwerking, het filteren of converteren van fouten, het herordenen of bufferen van notificaties, of elke andere situatie waarin u notificaties als data wilt verwerken en ze vervolgens naar hun oorspronkelijke formaat wilt terugbrengen.
---

# dematerialize - Herstel Notification-object

De `dematerialize` operator **converteert** een Notification-object naar een normale notificatie (next, error, complete). Het voert de omgekeerde transformatie van `materialize` uit en herstelt de gegevensgemaakte notificatie naar zijn oorspronkelijke vorm.

## 🔰 Basissyntax en werking

Converteert een stream van Notification-objecten terug naar een normale stream.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // Converteer naar Notification-object
    dematerialize()    // Herstel
  )
  .subscribe({
    next: v => console.log('Waarde:', v),
    complete: () => console.log('Voltooid')
  });
// Uitvoer:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Voltooid
```

[🌐 RxJS Officiële Documentatie - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 Typische gebruiksvoorbeelden

- **Notificaties herstellen na verwerking**: Herstel ze naar hun oorspronkelijke formaat na verwerking met materialize
- **Fouten filteren**: Alleen bepaalde fouten uitsluiten
- **Volgorde van notificaties herschikken**: Herstel na het sorteren van notificaties als data
- **Herstel na debugging**: Herstel normale werking na logging, etc.

## 🧪 Praktisch codevoorbeeld 1: Selectief filteren van fouten

Dit is een voorbeeld van het uitsluiten van alleen bepaalde fouten en de rest normaal verwerken.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// UI creatie
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'dematerialize - Fout filteren';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.appendChild(logItem);
}

// Stream met fouten
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('Negeerbare fout')),
  of(3, 4),
  throwError(() => new Error('Kritieke fout')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // Filter alleen "Negeerbare fout"
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('Negeerbare')) {
          addLog(`🔇 Genegeerd: ${errorMessage}`, '#fff9c4');
          return false;  // Sluit deze fout uit
        }
      }
      return true;
    }),
    dematerialize()  // Herstel naar oorspronkelijk formaat
  )
  .subscribe({
    next: v => addLog(`✅ Waarde: ${v}`, '#c8e6c9'),
    error: err => addLog(`❌ Fout: ${err.message}`, '#ffcdd2'),
    complete: () => addLog('Voltooid', '#e3f2fd')
  });
```

- "Negeerbare fouten" worden uitgesloten en de stream gaat door
- "Kritieke fouten" worden zoals gebruikelijk doorgegeven aan de error handler
- Selectieve afhandeling van fouten mogelijk

## 🧪 Praktisch codevoorbeeld 2: Vertraagde notificatie

Dit is een voorbeeld van het tijdelijk bufferen van een notificatie en deze vervolgens herstellen.

```ts
import { from, interval, take, delay } from 'rxjs';
import { materialize, dematerialize, bufferTime, concatMap } from 'rxjs';

// UI creatie
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'dematerialize - Bufferen en vertraging';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('nl-NL', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Start - geef waarden uit elke seconde, verwerk in batches elke 2 seconden');

interval(1000)
  .pipe(
    take(6),
    materialize(),
    bufferTime(2000),      // Buffer elke 2 seconden
    concatMap(notifications => {
      addLog2(`--- Verwerking van ${notifications.length} notificaties uit buffer ---`);
      return from(notifications).pipe(
        delay(500),        // Vertraag elke notificatie met 0,5 seconden
        dematerialize()    // Herstel naar oorspronkelijk formaat
      );
    })
  )
  .subscribe({
    next: v => addLog2(`Waarde: ${v}`),
    complete: () => addLog2('Voltooid')
  });
```

- Buffert notificaties elke 2 seconden
- Haal uit buffer en vertraag verwerking
- Herstel als oorspronkelijke stream met `dematerialize`

## 🆚 Relatie met materialize

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // Converteer naar Notification
    map(notification => {
      // Verwerk als Notification-object
      console.log('soort:', notification.kind);
      return notification;
    }),
    dematerialize()          // Herstel
  )
  .subscribe(v => console.log('Waarde:', v));
// Uitvoer:
// soort: N
// Waarde: 1
// soort: N
// Waarde: 2
// soort: N
// Waarde: 3
// soort: C
```

| Processtroom | Beschrijving |
|:---|:---|
| Originele stream | Normale waarde (next), fout (error), voltooiing (complete) |
| ↓ `materialize()` | Stream van Notification-object |
| Tussenverwerking | Verwerking en filtering als Notification |
| ↓ `dematerialize()` | Herstel naar normale stream |
| Eindstream | Normale waarde, fout, complete |

## ⚠️ Belangrijke opmerkingen

### 1. Foutnotificaties worden geconverteerd naar werkelijke fouten

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Converteer elke Observable naar notification-object met materialize()
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('Fout')).pipe(materialize()),
  of(2).pipe(materialize())  // Niet uitgevoerd na fout
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Waarde:', v),
    error: err => console.log('Fout:', err.message)
  });
// Uitvoer:
// Waarde: 1
// Fout: Fout
```

Wanneer een foutnotificatie wordt bereikt, wordt de stream onderbroken met een fout.

### 2. Voltooiingsnotificatie voltooit de stream

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Converteer elke Observable naar notification-object met materialize()
concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // Voltooiingsnotificatie
  of(3).pipe(materialize())   // Niet uitgevoerd na voltooiing
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Waarde:', v),
    complete: () => console.log('Voltooid')
  });
// Uitvoer:
// Waarde: 1
// Waarde: 2
// Voltooid
```

Er wordt geen waarde uitgegeven na de voltooiingsnotificatie.

### 3. Ongeldig Notification-object

De `dematerialize` verwacht een correct Notification-object.

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ Normale waarden doorgeven aan dematerialize veroorzaakt een fout
of(1, 2, 3)
  .pipe(
    dematerialize()  // Geen Notification-object
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Fout:', err.message)
  });
// Fout treedt op
```

## Praktische combinatievoorbeelden

```ts
import { interval, throwError, of, concat } from 'rxjs';
import { materialize, dematerialize, take, mergeMap, map } from 'rxjs';

// Voorbeeld van het converteren van fouten naar waarschuwingen
interval(500)
  .pipe(
    take(10),
    mergeMap(value => {
      // Genereer alleen fout wanneer 5
      if (value === 5) {
        return throwError(() => new Error(`Fout bij waarde ${value}`));
      }
      return of(value);
    }),
    materialize(),
    map(notification => {
      // Converteer fouten naar waarschuwingsberichten
      if (notification.kind === 'E') {
        console.warn('Waarschuwing:', notification.error?.message);
        // Geef speciale waarde uit in plaats van fout (gegenereerd door materialize())
        return { kind: 'N' as const, value: -1 };
      }
      return notification;
    }),
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Waarde:', v),
    error: err => console.error('Fout:', err),  // Niet aangeroepen
    complete: () => console.log('Voltooid')
  });
// Uitvoer:
// Waarde: 0, 1, 2, 3, 4
// Waarschuwing: Fout bij waarde 5
// Waarde: -1  (in plaats van fout)
// Waarde: 6, 7, 8, 9
// Voltooid
```

## 📚 Gerelateerde operators

- **[materialize](./materialize)** - Converteer notificatie naar Notification-object
- **[catchError](/nl/guide/error-handling/retry-catch)** - Foutafhandeling
- **[retry](./retry)** - Opnieuw proberen bij fout

## ✅ Samenvatting

De `dematerialize` operator herstelt het Notification-object naar een normale notificatie.

- ✅ Omgekeerde `materialize` conversie
- ✅ Herstelt de notificatie naar zijn oorspronkelijke formaat na verwerking
- ✅ Maakt filtering en conversie van fouten mogelijk
- ✅ Kan worden gebruikt om notificaties te herordenen of te bufferen
- ⚠️ Foutnotificaties werken als werkelijke fouten
- ⚠️ Voltooiingsnotificatie voltooit stream
- ⚠️ Vereist correct Notification-object
