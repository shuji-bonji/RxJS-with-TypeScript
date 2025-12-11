---
description: De subscribeOn operator regelt wanneer een abonnement op Observable begint met de gespecificeerde scheduler en wijzigt de uitvoeringscontext van de gehele stream.
---

# subscribeOn - Regel wanneer abonnement begint

De `subscribeOn` operator regelt Observable's **starttiming van abonnement en uitvoeringscontext met de gespecificeerde scheduler**. Het beïnvloedt de uitvoeringstiming van de gehele stream.

## 🔰 Basissyntax en werking

Asynchroon maken van de start van een abonnement door een scheduler te specificeren.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Waarde:', v));

console.log('Einde');

// Uitvoer:
// Start
// Einde
// Waarde: 1
// Waarde: 2
// Waarde: 3
```

De start van het abonnement zelf wordt asynchroon gemaakt, dus de aanroep van `subscribe()` keert onmiddellijk terug.

[🌐 RxJS Officiële Documentatie - subscribeOn](https://rxjs.dev/api/index/function/subscribeOn)

## 💡 Typische gebruiksvoorbeelden

- **Zware initialisatieprocessen asynchroon maken**: Het starten van data laden uitstellen, etc.
- **UI-bevriezing voorkomen**: Asynchroon abonnementen starten om responsiviteit te behouden
- **Verwerking prioriteren**: Starttiming van meerdere streams regelen
- **Timing controle bij testen**: Regelen met TestScheduler

## 🧪 Praktisch codevoorbeeld 1: Zware initialisatieverwerking asynchroon maken

Dit is een voorbeeld van het asynchroon starten van data lezen en initialisatie.

```ts
import { Observable, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// UI creatie
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'subscribeOn - Zware initialisatieverwerking';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const timestamp = now.toLocaleTimeString('nl-NL', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

// Simuleer zware initialisatieverwerking
const heavyInit$ = new Observable<string>(subscriber => {
  addLog('Data laden gestart...', '#fff9c4');

  // Simuleer zware verwerking
  let sum = 0;
  for (let i = 0; i < 10000000; i++) {
    sum += i;
  }

  addLog('Data laden voltooid', '#c8e6c9');
  subscriber.next(`Resultaat: ${sum}`);
  subscriber.complete();
});

addLog('Abonnement starten (UI bedienbaar)', '#e3f2fd');

heavyInit$
  .pipe(
    subscribeOn(asyncScheduler)  // Abonnementstart asynchroon maken
  )
  .subscribe({
    next: result => addLog(`Ontvangen: ${result}`, '#c8e6c9'),
    complete: () => addLog('Voltooid', '#e3f2fd')
  });

addLog('Na abonnementsverzoek (uitvoering gaat direct door)', '#e3f2fd');
```

- Abonnementstart is asynchroon, UI reageert onmiddellijk
- Zware verwerking wordt asynchroon uitgevoerd
- Hoofdthread wordt niet geblokkeerd

## 🧪 Praktisch codevoorbeeld 2: Prioriteitscontrole van meerdere streams

Dit is een voorbeeld van het regelen van de starttiming van meerdere streams.

```ts
import { interval, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn, take, tap } from 'rxjs';

// UI creatie
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'subscribeOn - Prioriteitscontrole';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('nl-NL', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '12px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Start', '#e3f2fd');

// Hoge prioriteit taak (asapScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asapScheduler),
    tap(v => addLog2(`Hoge prioriteit: ${v}`, '#c8e6c9'))
  )
  .subscribe();

// Normale prioriteit taak (asyncScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asyncScheduler),
    tap(v => addLog2(`Normale prioriteit: ${v}`, '#fff9c4'))
  )
  .subscribe();

addLog2('Abonnementsverzoek voltooid', '#e3f2fd');
```

- Verschillende schedulers regelen prioriteiten
- `asapScheduler` start uitvoering eerder dan `asyncScheduler`

## 🆚 Verschillen met observeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

// observeOn voorbeeld
console.log('=== observeOn ===');
console.log('1: Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('2: tap (sync)')),
    observeOn(asyncScheduler),
    tap(() => console.log('4: tap (async)'))
  )
  .subscribe(() => console.log('5: subscribe'));

console.log('3: Einde');

// subscribeOn voorbeeld
console.log('\n=== subscribeOn ===');
console.log('1: Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('3: tap (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(() => console.log('4: subscribe'));

console.log('2: Einde');
```

**Belangrijkste verschillen**:

| Item | observeOn | subscribeOn |
|:---|:---|:---|
| **Bereik van effecten** | Alleen volgende verwerking | Gehele stream |
| **Controledoel** | Timing van waarde publiceren | Timing van abonnementstart |
| **Positionering** | Belangrijk (gedrag verandert afhankelijk van waar je het plaatst) | Overal hetzelfde |
| **Meervoudig gebruik** | Laatste wordt toegepast | Eerste wordt toegepast |

> [!NOTE]
> Voor meer informatie over `observeOn`, zie [observeOn](./observeOn.md).

## ⚠️ Belangrijke opmerkingen

### 1. Plaatsingspositie heeft geen effect

`subscribeOn` heeft hetzelfde effect ongeacht waar je het in de pipeline plaatst.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, map } from 'rxjs';

// Patroon 1: Eerst
of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),
    map(x => x * 2)
  )
  .subscribe();

// Patroon 2: Laatst
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Beide werken hetzelfde
```

### 2. Meerdere subscribeOn's passen de eerste toe

```ts
import { of, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),  // Deze wordt gebruikt
    subscribeOn(asapScheduler)    // Deze wordt genegeerd
  )
  .subscribe();
```

De eerste `subscribeOn` scheduler (`asyncScheduler`) wordt gebruikt.

### 3. Sommige Observables hebben geen effect

Observables met hun eigen scheduler, zoals `interval` en `timer`, worden niet beïnvloed door `subscribeOn`.

```ts
import { interval, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// ❌ subscribeOn heeft geen effect
interval(1000)
  .pipe(
    subscribeOn(asyncScheduler)  // interval gebruikt zijn eigen scheduler
  )
  .subscribe();

// ✅ Specificeer scheduler in interval argument
interval(1000, asyncScheduler)
  .subscribe();
```

## Praktische combinatievoorbeelden

```ts
import { of, asyncScheduler, animationFrameScheduler } from 'rxjs';
import { subscribeOn, observeOn, map, tap } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Tap 1 (async)')),
    subscribeOn(asyncScheduler),        // Abonnementstart asynchroon maken
    map(x => x * 2),
    observeOn(animationFrameScheduler), // Synchroniseer waarde-uitgifte met animatieframe
    tap(() => console.log('Tap 2 (animatieframe)'))
  )
  .subscribe(v => console.log('Waarde:', v));

console.log('Einde');

// Uitvoeringsvolgorde:
// Start
// Einde
// Tap 1 (async)
// Tap 1 (async)
// Tap 1 (async)
// Tap 2 (animatieframe)
// Waarde: 2
// ... (vervolg hieronder)
```

## Gebruiksrichtlijnen

### Geval 1: U wilt de start van abonnementen uitstellen
```ts
// → gebruik subscribeOn
of(data)
  .pipe(subscribeOn(asyncScheduler))
  .subscribe();
```

### Geval 2: Ik wil een specifiek proces asynchroon maken
```ts
// → gebruik observeOn
of(data)
  .pipe(
    map(zware verwerking),
    observeOn(asyncScheduler),  // Alleen asynchroon maken na zware verwerking
    map(lichte verwerking)
  )
  .subscribe();
```

### Geval 3: Gehele proces asynchroon maken + een deel verder controleren
```ts
// → gebruik subscribeOn + observeOn samen
of(data)
  .pipe(
    subscribeOn(asyncScheduler),           // Gehele proces asynchroon maken
    map(verwerking 1),
    observeOn(animationFrameScheduler),    // Wijzigen voor animatie
    map(verwerking 2)
  )
  .subscribe();
```

## 📚 Gerelateerde operators

- **[observeOn](./observeOn)** - Regelt wanneer waarden worden uitgegeven
- **[delay](./delay)** - Vaste tijdvertraging

## 📖 Gerelateerde documenten

- **[Controle van asynchrone verwerking](/nl/guide/schedulers/async-control.md)** - Scheduler basis
- **[Soorten en gebruik van schedulers](/nl/guide/schedulers/types.md)** - Details van elke scheduler

## ✅ Samenvatting

De `subscribeOn` operator regelt de timing en uitvoeringscontext voor het starten van abonnementen.

- ✅ Maakt abonnementstart asynchroon voor gehele stream
- ✅ Nuttig voor het asynchroon maken van zware initialisatieprocessen
- ✅ Nuttig voor het voorkomen van UI-bevriezing
- ✅ Positie van plaatsing heeft geen effect
- ⚠️ Bij meerdere Observables wordt de eerste toegepast
- ⚠️ Niet effectief voor sommige Observables
- ⚠️ Ander doel dan `observeOn`
