---
description: materialize is een RxJS utility operator die Observable-notificaties (next, error, complete) converteert naar Notification-objecten. Het is ideaal voor situaties waarin u de notificatie zelf wilt manipuleren, zoals het behandelen van fouten als data, debugging en logging van notificaties, het vastleggen van meta-informatie, etc. dematerialize stelt u in staat het oorspronkelijke formaat te herstellen en type-veilige notificatieverwerking met TypeScript type-inferentie.
---

# materialize - Objectificeer notificaties

De `materialize` operator converteert Observable **notificaties (next, error, complete) naar Notification-objecten**. Dit maakt het mogelijk om niet alleen waarden maar ook fouten en voltooiingen als data te behandelen.

## 🔰 Basissyntax en werking

Converteert een normale stream naar een stream van Notification-objecten.

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// Uitvoer:
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

De `kind` eigenschap van het Notification-object:
- `'N'`: next (waarde uitgegeven)
- `'E'`: error
- `'C'`: complete

[🌐 RxJS Officiële Documentatie - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 Typische gebruiksvoorbeelden

- **Fouten als data behandelen**: Behandel fouten als onderdeel van de stream
- **Debugging en logging**: Gedetailleerde tracking van notificaties
- **Meta-informatie vastleggen**: Registreer wanneer en welke soort notificaties optreden
- **Streams met fouten combineren**: Behandel fouten in meerdere streams uniform

## 🧪 Praktisch codevoorbeeld 1: Fouten als data behandelen

Dit voorbeeld toont hoe fouten die normaal een stream zouden onderbreken als data te behandelen en door te gaan.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// UI creatie
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'materialize - Fouten als data';
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

// Normale foutafhandeling (stream onderbroken)
addLog('--- Normale foutafhandeling ---', '#e3f2fd');
concat(
  of(1, 2),
  throwError(() => new Error('Fout opgetreden')),
  of(3, 4)  // Hier niet uitgevoerd
).subscribe({
  next: v => addLog(`Waarde: ${v}`, '#c8e6c9'),
  error: err => addLog(`❌ Fout: ${err.message}`, '#ffcdd2'),
  complete: () => addLog('Voltooid', '#e3f2fd')
});

// Met materialize (stream gaat door)
setTimeout(() => {
  addLog('--- Met materialize ---', '#e3f2fd');

  concat(
    of(1, 2),
    throwError(() => new Error('Fout opgetreden')),
    of(3, 4)
  )
    .pipe(
      materialize(),
      map(notification => {
        if (notification.kind === 'N') {
          return `Waarde: ${notification.value}`;
        } else if (notification.kind === 'E') {
          return `Fout (als data): ${notification.error?.message}`;
        } else {
          return 'Voltooid';
        }
      })
    )
    .subscribe({
      next: msg => {
        const color = msg.includes('Fout') ? '#fff9c4' : '#c8e6c9';
        addLog(msg, color);
      },
      complete: () => addLog('Stream voltooid', '#e3f2fd')
    });
}, 1000);
```

- Normale fouten onderbreken de stream
- Met `materialize` worden fouten behandeld als data en gaat de stream door

## 🧪 Praktisch codevoorbeeld 2: Debug logging

Hier is een voorbeeld dat alle notificaties in detail logt.

```ts
import { interval, throwError } from 'rxjs';
import { materialize, take, mergeMap } from 'rxjs';

// UI creatie
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'materialize - Debug logging';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '250px';
output2.style.overflow = 'auto';
output2.style.fontFamily = 'monospace';
output2.style.fontSize = '12px';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('nl-NL', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.marginBottom = '2px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

interval(500)
  .pipe(
    take(5),
    mergeMap(value => {
      // Genereer fout wanneer waarde 3 is
      if (value === 3) {
        return throwError(() => new Error('Fout bij waarde 3'));
      }
      return of(value);
    }),
    materialize()
  )
  .subscribe({
    next: notification => {
      switch (notification.kind) {
        case 'N':
          addLog2(`[NEXT] waarde: ${notification.value}`);
          break;
        case 'E':
          addLog2(`[ERROR] ${notification.error?.message}`);
          break;
        case 'C':
          addLog2('[COMPLETE]');
          break;
      }
    },
    complete: () => {
      addLog2('--- Observer voltooid ---');
    }
  });
```

- Uniforme logging van alle notificatietypes (next, error, complete)
- Volgt de volgorde van notificaties met tijdstempels
- Nuttig voor debugging en monitoring

## 🆚 Vergelijking met normale streams

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

// Normale stream
of(1, 2, 3).subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('Voltooid')
});
// Uitvoer:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Voltooid

// Met materialize
of(1, 2, 3)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notificatie:', n),
    complete: () => console.log('Voltooid')
  });
// Uitvoer:
// Notificatie: Notification { kind: 'N', value: 1, ... }
// Notificatie: Notification { kind: 'N', value: 2, ... }
// Notificatie: Notification { kind: 'N', value: 3, ... }
// Notificatie: Notification { kind: 'C', ... }
// Voltooid
```

## Notification-object manipuleren

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // Eigenschappen van Notification-object
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // Heeft waarde
        value: notification.value,         // Waarde (voor next)
        error: notification.error          // Fout (voor error)
      };
    })
  )
  .subscribe(console.log);
// Uitvoer:
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ Belangrijke opmerkingen

### 1. Fouten onderbreken de stream niet

Bij gebruik van `materialize` worden fouten behandeld als data en wordt de stream niet onderbroken.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('Fout')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notificatie:', n.kind),
    error: () => console.log('Error handler'),  // Niet aangeroepen
    complete: () => console.log('Voltooid')
  });
// Uitvoer:
// Notificatie: N
// Notificatie: E  ← Fouten worden ook behandeld als next
// Voltooid
```

### 2. Combinatie met dematerialize

Streams getransformeerd met `materialize` kunnen worden hersteld met `dematerialize`.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // Hier wat verwerking
    dematerialize()  // Herstel
  )
  .subscribe(console.log);
// Uitvoer: 1, 2, 3
```

### 3. Prestatie-impact

Er is overhead bij het genereren van Notification-objecten. Gebruik alleen wanneer nodig in een productieomgeving.

## 📚 Gerelateerde operators

- **[dematerialize](./dematerialize)** - Zet Notification-object terug naar normale notificatie
- **[tap](./tap)** - Voer een bijwerking uit (voor debugging doeleinden)
- **[catchError](/nl/guide/error-handling/retry-catch)** - Foutafhandeling

## ✅ Samenvatting

De `materialize` operator converteert een notificatie naar een Notification-object.

- ✅ Kan fouten behandelen als data
- ✅ Nuttig voor debugging en logging
- ✅ Kan meta-informatie over notificaties vastleggen
- ✅ Kan ongedaan worden gemaakt met `dematerialize`
- ⚠️ Fouten zullen de stream niet langer onderbreken
- ⚠️ Let op prestatie-overhead
