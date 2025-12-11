---
description: De skipWhile operator slaat waarden over terwijl de gespecificeerde voorwaarde waar is en geeft alle volgende waarden uit vanaf het punt dat de voorwaarde onwaar wordt. Het is nuttig wanneer u een stream wilt controleren met een dynamische startvoorwaarde.
---

# skipWhile - Sla waarden over terwijl voorwaarde voldaan is

De `skipWhile` operator blijft waarden **overslaan terwijl de gespecificeerde voorwaarde waar is**, en geeft **alle volgende waarden** uit vanaf het punt dat de voorwaarde `false` wordt.

## 🔰 Basissyntax en gebruik

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0 tot 9

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

**Werkingsstroom**:
1. 0 wordt uitgegeven → `0 < 5` is `true` → Overslaan
2. 1 wordt uitgegeven → `1 < 5` is `true` → Overslaan
3. 2 wordt uitgegeven → `2 < 5` is `true` → Overslaan
4. 3 wordt uitgegeven → `3 < 5` is `true` → Overslaan
5. 4 wordt uitgegeven → `4 < 5` is `true` → Overslaan
6. 5 wordt uitgegeven → `5 < 5` is `false` → Start uitvoer
7. 6 en daarna → Allemaal uitvoer (voorwaarde wordt niet opnieuw geëvalueerd)

[🌐 RxJS Officiële Documentatie - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 Typische gebruikspatronen

- **Sla initiële onnodige data over**: Sluit data tijdens opwarmperiode uit
- **Sla over tot drempelwaarde bereikt is**: Wacht tot specifieke voorwaarden voldaan zijn
- **Sla koptekstrijen over**: Sluit CSV-kopteksten uit, etc.
- **Sla voorbereidingsperiode over**: Wacht tot systeem gereed is

## 🧠 Praktisch codevoorbeeld 1: Sla sensor opwarmperiode over

Voorbeeld van het overslaan van initiële data tot de sensor stabiliseert.

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

// Maak UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Temperatuursensor Monitoring';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginBottom = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#fff3e0';
status.style.border = '1px solid #FF9800';
status.textContent = '🔄 Sensor voorbereiden... (Meting start wanneer temperatuur >= 20°C)';
container.appendChild(status);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

let isWarmedUp = false;

// Temperatuursensor simulatie (geleidelijk opwarmen)
interval(500).pipe(
  take(20),
  map(i => {
    // Start lage temperatuur, verhoog geleidelijk
    const baseTemp = 15 + i * 0.5;
    const noise = (Math.random() - 0.5) * 2;
    return baseTemp + noise;
  }),
  skipWhile(temp => temp < 20) // Sla onder 20°C over
).subscribe({
  next: temp => {
    // Update status wanneer eerste waarde aankomt
    if (!isWarmedUp) {
      isWarmedUp = true;
      status.textContent = '✅ Sensor gereed (Meting gestart)';
      status.style.backgroundColor = '#e8f5e9';
      status.style.borderColor = '#4CAF50';
    }

    const log = document.createElement('div');
    log.style.padding = '5px';
    log.style.marginBottom = '3px';
    log.style.backgroundColor = temp > 25 ? '#ffebee' : '#f1f8e9';
    log.textContent = `[${new Date().toLocaleTimeString()}] Temperatuur: ${temp.toFixed(1)}°C`;
    output.insertBefore(log, output.firstChild);

    // Toon maximaal 10 items
    while (output.children.length > 10) {
      output.removeChild(output.lastChild!);
    }
  },
  complete: () => {
    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = 'Meting voltooid';
    container.appendChild(summary);
  }
});
```

- Data wordt overgeslagen terwijl de sensor onder 20°C is.
- Alle data wordt geregistreerd vanaf het punt dat het 20°C of hoger bereikt.

## 🎯 Praktisch codevoorbeeld 2: Gebeurtenisverwerking na gereed

Voorbeeld van het overslaan van gebeurtenissen tot systeeminitialisatie voltooit.

```ts
import { fromEvent, merge, Subject } from 'rxjs';
import { skipWhile, map, tap } from 'rxjs';

// Maak UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Gebeurtenis Verwerkingssysteem';
container.appendChild(title);

const initButton = document.createElement('button');
initButton.textContent = 'Initialisatie Voltooien';
initButton.style.marginRight = '10px';
container.appendChild(initButton);

const eventButton = document.createElement('button');
eventButton.textContent = 'Gebeurtenis Vuren';
container.appendChild(eventButton);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
statusDiv.style.padding = '10px';
statusDiv.style.backgroundColor = '#ffebee';
statusDiv.style.border = '1px solid #f44336';
statusDiv.innerHTML = '<strong>⏸️ Systeem Niet Geïnitialiseerd</strong><br>Gebeurtenissen worden overgeslagen';
container.appendChild(statusDiv);

const eventLog = document.createElement('div');
eventLog.style.marginTop = '10px';
eventLog.style.border = '1px solid #ccc';
eventLog.style.padding = '10px';
eventLog.style.minHeight = '100px';
container.appendChild(eventLog);

// Initialisatiestatus
let isInitialized = false;
const initSubject = new Subject<boolean>();

// Initialisatieknop
fromEvent(initButton, 'click').subscribe(() => {
  if (!isInitialized) {
    isInitialized = true;
    initSubject.next(true);
    statusDiv.style.backgroundColor = '#e8f5e9';
    statusDiv.style.borderColor = '#4CAF50';
    statusDiv.innerHTML = '<strong>✅ Systeem Geïnitialiseerd</strong><br>Gebeurtenissen verwerken';
    initButton.disabled = true;
  }
});

// Gebeurtenisverwerking (sla over tot geïnitialiseerd)
let eventCount = 0;
fromEvent(eventButton, 'click').pipe(
  map(() => {
    eventCount++;
    return {
      id: eventCount,
      timestamp: new Date(),
      initialized: isInitialized
    };
  }),
  tap(event => {
    if (!event.initialized) {
      const skipLog = document.createElement('div');
      skipLog.style.padding = '5px';
      skipLog.style.marginBottom = '3px';
      skipLog.style.color = '#999';
      skipLog.textContent = `⏭️ Gebeurtenis #${event.id} overgeslagen (niet geïnitialiseerd)`;
      eventLog.insertBefore(skipLog, eventLog.firstChild);
    }
  }),
  skipWhile(event => !event.initialized)
).subscribe(event => {
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.marginBottom = '3px';
  log.style.backgroundColor = '#e8f5e9';
  log.style.border = '1px solid #4CAF50';
  log.innerHTML = `
    <strong>✅ Gebeurtenis #${event.id} Verwerkt</strong>
    [${event.timestamp.toLocaleTimeString()}]
  `;
  eventLog.insertBefore(log, eventLog.firstChild);

  // Toon maximaal 10 items
  while (eventLog.children.length > 10) {
    eventLog.removeChild(eventLog.lastChild!);
  }
});
```

- Alle gebeurtenissen worden overgeslagen tot het systeem is geïnitialiseerd.
- Na voltooiing van initialisatie worden alle gebeurtenissen verwerkt.

## 🆚 Vergelijking met vergelijkbare operators

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs';

const numbers$ = range(0, 10); // 0 tot 9

// skipWhile: Sla over terwijl voorwaarde voldaan is, dan alles uitvoeren
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9

// takeWhile: Neem alleen terwijl voorwaarde voldaan is
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// skip: Sla eerste N waarden over
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9

// filter: Laat alleen waarden door die aan voorwaarde voldoen (geëvalueerd voor alle)
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

| Operator | Gedrag | Voorwaarde opnieuw evalueren | Voltooiingstiming |
|:---|:---|:---|:---|
| `skipWhile(predicate)` | Sla over terwijl voorwaarde voldaan is | Nee (eindigt zodra onwaar) | Wanneer bronstream voltooit |
| `takeWhile(predicate)` | Neem terwijl voorwaarde voldaan is | Elke keer | Wanneer voorwaarde onwaar wordt |
| `skip(n)` | Sla eerste n waarden over | Geen (op aantal gebaseerd) | Wanneer bronstream voltooit |
| `filter(predicate)` | Laat alleen overeenkomende waarden door | **Elke keer** | Wanneer bronstream voltooit |

**Visuele verschillen**:

```
Invoer: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 overgeslagen] | 5, 4, 3, 2, 1, 0
                          ^Alle uitvoer nadat voorwaarde onwaar wordt

filter(n => n >= 5):
[0,1,2,3,4 uitgesloten] 5 [4,3,2,1,0 uitgesloten]
                        ^Alleen overeenkomende waarden uitvoeren (elke keer geëvalueerd)

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [negeer alles na 5 en voltooi]
```

## ⚠️ Belangrijke opmerkingen

### 1. Voorwaarde wordt niet opnieuw geëvalueerd zodra onwaar

Dit is het grootste verschil met `filter`.

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: Zodra voorwaarde onwaar wordt, alle volgende waarden uitvoeren
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// Output: skipWhile: 5, 4, 3, 2, 1 (alles na 5)

// filter: Evalueer voorwaarde elke keer
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// Output: filter: 5 (alleen 5)
```

### 2. Als voorwaarde onwaar is vanaf het begin

Als de voorwaarde `false` is vanaf het begin, worden alle waarden uitgevoerd.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(5, 5).pipe( // 5 tot 9
  skipWhile(n => n < 3) // Voorwaarde is onwaar vanaf start
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9 (alle uitvoer)
```

### 3. Als alle waarden aan voorwaarde voldoen

Als alle waarden aan de voorwaarde voldoen, wordt niets uitgevoerd.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(0, 5).pipe( // 0 tot 4
  skipWhile(n => n < 10) // Alle waarden voldoen aan voorwaarde
).subscribe({
  next: console.log,
  complete: () => console.log('Voltooid (niets uitgevoerd)')
});
// Output: Voltooid (niets uitgevoerd)
```

### 4. TypeScript types

`skipWhile` verandert het type niet.

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// Type blijft Observable<User>
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// Output: Charlie (ID: 3), Dave (ID: 4)
```

## 💡 Praktische combinatiepatronen

### Patroon 1: Sla koptekstrij over

Sla CSV-koptekstrijen over

```ts
import { from } from 'rxjs';
import { skipWhile, map } from 'rxjs';

const csvLines$ = from([
  'Naam,Leeftijd,Stad',     // Koptekstrij
  'Alice,25,Tokyo',
  'Bob,30,Osaka',
  'Charlie,35,Kyoto'
]);

let isFirstLine = true;

csvLines$.pipe(
  skipWhile(() => {
    if (isFirstLine) {
      isFirstLine = false;
      return true; // Sla eerste rij over (koptekst)
    }
    return false;
  }),
  map(line => {
    const [name, age, city] = line.split(',');
    return { name, age: Number(age), city };
  })
).subscribe(console.log);
// Output:
// { name: 'Alice', age: 25, city: 'Tokyo' }
// { name: 'Bob', age: 30, city: 'Osaka' }
// { name: 'Charlie', age: 35, city: 'Kyoto' }
```

### Patroon 2: Tijdstempel-gebaseerd filteren

Verwerk alleen data na specifieke tijd

```ts
import { from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface LogEntry {
  timestamp: Date;
  message: string;
}

const startTime = new Date('2025-01-01T12:00:00');

const logs$ = from([
  { timestamp: new Date('2025-01-01T10:00:00'), message: 'Log 1' },
  { timestamp: new Date('2025-01-01T11:00:00'), message: 'Log 2' },
  { timestamp: new Date('2025-01-01T12:00:00'), message: 'Log 3' },
  { timestamp: new Date('2025-01-01T13:00:00'), message: 'Log 4' }
] as LogEntry[]);

logs$.pipe(
  skipWhile(log => log.timestamp < startTime)
).subscribe(log => {
  console.log(`[${log.timestamp.toISOString()}] ${log.message}`);
});
// Output:
// [2025-01-01T12:00:00.000Z] Log 3
// [2025-01-01T13:00:00.000Z] Log 4
```

### Patroon 3: Status-gebaseerd overslaan

Sla over tot systeem gereed is

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

interface SystemState {
  tick: number;
  isReady: boolean;
  data: number;
}

// Systeemstatus simulatie
interval(500).pipe(
  take(10),
  map(i => ({
    tick: i,
    isReady: i >= 3, // Gereed na 3 seconden
    data: Math.floor(Math.random() * 100)
  } as SystemState)),
  skipWhile(state => !state.isReady)
).subscribe(state => {
  console.log(`Tick ${state.tick}: data=${state.data}`);
});
// Output: Alleen data vanaf Tick 3 en verder
```

## 📚 Gerelateerde operators

- **[takeWhile](/nl/guide/operators/filtering/takeWhile)** - Neem waarden alleen terwijl voorwaarde voldaan is
- **[skip](/nl/guide/operators/filtering/skip)** - Sla eerste N waarden over
- **[skipLast](/nl/guide/operators/filtering/skipLast)** - Sla laatste N waarden over
- **[skipUntil](/nl/guide/operators/filtering/skipUntil)** - Sla over tot andere Observable vuurt
- **[filter](/nl/guide/operators/filtering/filter)** - Laat alleen waarden door die aan voorwaarde voldoen

## Samenvatting

De `skipWhile` operator slaat waarden over terwijl een voorwaarde waar is en geeft alle volgende waarden uit vanaf het punt dat de voorwaarde onwaar wordt.

- ✅ Ideaal voor het overslaan van initiële onnodige data
- ✅ Voorwaarde wordt niet opnieuw geëvalueerd zodra deze onwaar wordt
- ✅ Nuttig voor het overslaan van opwarm- of voorbereidingsperiodes
- ✅ Kan worden gebruikt om koptekstrijen over te slaan
- ⚠️ In tegenstelling tot `filter`, wordt voorwaarde slechts één keer geëvalueerd
- ⚠️ Als alle waarden aan voorwaarde voldoen, wordt niets uitgevoerd
- ⚠️ Gaat door tot bronstream voltooit
