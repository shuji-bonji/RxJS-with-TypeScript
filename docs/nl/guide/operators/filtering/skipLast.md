---
description: "De skipLast operator is een RxJS filter operator die de laatste N waarden van de Observable stream overslaat en alleen de waarden ervoor uitvoert."
---

# skipLast - de laatste N waarden overslaan

De `skipLast` operator **slaat de laatste N waarden van de bron Observable over** en voert alleen de voorgaande waarden uit. Het houdt de laatste N waarden in de buffer totdat de stream is voltooid en voert de rest uit.

## 🔰 Basissyntaxis en gebruik

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0van (tot)9naar

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Uitgang: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wordt overgeslagen)
```

**Stroom van de bewerking**:.
1. de stroom geeft 0, 1, 2, ... afgegeven.
2. bewaar de laatste 3 waarden (7, 8, 9) in een buffer
3. uitvoerwaarden groter dan buffergrootte (0-6)
4. bij voltooiing van de stream worden de bufferwaarden (7, 8, 9) niet uitgevoerd maar weggegooid

[🌐 Officiële RxJS documentatie - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Typisch gebruikspatroon.

- **Uitsluiten laatste gegevens**: uitsluiten laatste gegevens die niet zijn afgerond
- **Batchverwerking**: niet-afgeronde gegevens uitsluiten voordat de verwerking is voltooid
- **Gegevens valideren**: als validatie vereist is voor opeenvolgende waarden.
- **Uitgestelde verwerking van definitieve gegevens**: als de laatste N gegevens niet definitief zijn.

## Praktisch codevoorbeeld 1: gegevensverwerkingspijplijn

Dit is een voorbeeld van het overslaan van de laatste niet-afgeronde gegevens bij gegevensverwerking.

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// UIaanmaken
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Pijplijn voor gegevensverwerking';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = 'De laatste2gevallen (niet-afgeronde gegevens) worden overgeslagen en verwerkt';
container.appendChild(description);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

interface DataPoint {
  id: number;
  value: number;
  status: 'processing' | 'confirmed' | 'skipped';
}

// De gegevensstroom (10geval)
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// 0.5Elke seconde gegevens publiceren
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // De laatste2Het laatste geval overslaan
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ De</strong>
      ID: ${item.id} |
      Waarde: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // Overgeslagen items tonen
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ Overslaan</strong>
        ID: ${item.id} |
        Waarde: ${item.value} |
        (onbevestigde gegevens)
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `Verwerking voltooid: ${data.length - 2}Item bevestigd,2Items overgeslagen`;
    output.appendChild(summary);
  }
});
```

- Gegevens worden opeenvolgend verwerkt, maar de laatste twee items worden behandeld als niet-afgerond en overgeslagen.
- Na voltooiing worden de overgeslagen items ook weergegeven.

## Praktisch codevoorbeeld 2: Logboek filteren

Dit is een voorbeeld van het overslaan van de laatste niet-afgeronde logs uit een logboekstroom.

```ts
import { interval } from 'rxjs';
import { skipLast, map, take } from 'rxjs';

// UIaanmaken
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Logboek controle';
container.appendChild(title);

const info = document.createElement('div');
info.style.marginBottom = '10px';
info.textContent = 'Laatste3case logs worden overgeslagen omdat ze wachten op afronding';
info.style.color = '#666';
container.appendChild(info);

const confirmedLogs = document.createElement('div');
confirmedLogs.innerHTML = '<strong>📋 Logboeken bevestigd:</strong>';
confirmedLogs.style.marginBottom = '10px';
container.appendChild(confirmedLogs);

const confirmedList = document.createElement('div');
confirmedList.style.border = '1px solid #4CAF50';
confirmedList.style.padding = '10px';
confirmedList.style.backgroundColor = '#f1f8e9';
confirmedList.style.minHeight = '100px';
container.appendChild(confirmedList);

const pendingLogs = document.createElement('div');
pendingLogs.innerHTML = '<strong>⏳ Logs wachten op bevestiging (overgeslagen):</strong>';
pendingLogs.style.marginTop = '10px';
pendingLogs.style.marginBottom = '10px';
container.appendChild(pendingLogs);

const pendingList = document.createElement('div');
pendingList.style.border = '1px solid #FF9800';
pendingList.style.padding = '10px';
pendingList.style.backgroundColor = '#fff3e0';
pendingList.style.minHeight = '60px';
container.appendChild(pendingList);

interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error';
  message: string;
}

// Gegenereerde logbestanden (totaal12Logs gegenereerd (totaal,1elke seconde)
const logs$ = interval(1000).pipe(
  take(12),
  map(i => {
    const levels: ('info' | 'warn' | 'error')[] = ['info', 'warn', 'error'];
    const messages = [
      'Inloggen gebruiker',
      'Start gegevensverwerving',
      'Cache bijwerken',
      'Fout in verbinding',
      'Opnieuw proberen',
      'Gegevensverwerking voltooid'
    ];
    return {
      id: i,
      timestamp: new Date(),
      level: levels[Math.floor(Math.random() * levels.length)],
      message: messages[Math.floor(Math.random() * messages.length)]
    } as LogEntry;
  })
);

const allLogs: LogEntry[] = [];

// Alles loggen (voor bevestiging)
logs$.subscribe(log => {
  allLogs.push(log);
});

// De laatste3Bevestigde logboeken weergeven, gevallen overslaan
logs$.pipe(
  skipLast(3)
).subscribe({
  next: log => {
    const logDiv = document.createElement('div');
    logDiv.style.padding = '3px';
    logDiv.style.marginBottom = '3px';
    const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
    logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
    confirmedList.appendChild(logDiv);
  },
  complete: () => {
    // De laatste3Het geval weergeven (overgeslagen logs)
    const skippedLogs = allLogs.slice(-3);
    skippedLogs.forEach(log => {
      const logDiv = document.createElement('div');
      logDiv.style.padding = '3px';
      logDiv.style.marginBottom = '3px';
      const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
      logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
      pendingList.appendChild(logDiv);
    });
  }
});
```

- Logs worden achtereenvolgens toegevoegd, maar de drie laatste logs worden overgeslagen omdat ze wachten op voltooiing.
- Na voltooiing worden de overgeslagen logboeken ook weergegeven.

## Vergelijking met vergelijkbare operators

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0van (tot)9naar

// skipLast: De laatsteNEén item overslaan
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Uitgang: 0, 1, 2, 3, 4, 5, 6

// takeLast: De laatsteNSlechts één item ophalen
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uitgang: 7, 8, 9

// skip: Eerste logboekvermeldingNEén item overslaan
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Uitgang: 3, 4, 5, 6, 7, 8, 9
```

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0van (tot)9naar

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Uitgang: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wordt overgeslagen)
```

**visuele verschillen**:.

Invoer: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 Overslaan]
                                   ^De laatste3Stukken

takeLast(3): [0~6 Overslaan] | 7, 8, 9
                             ^De laatste3Slechts 1 stuk

skip(3): [0, 1, 2 Overslaan] | 3, 4, 5, 6, 7, 8, 9
          ^Eerste logboekvermelding3Stukken
```

## ⚠️ Opmerkingen.

### 1. werkt met oneindige streams

`skipLast` werkt niet zoals bedoeld met oneindige streams omdat het de laatste N niet kan identificeren tot voltooiing.

```

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Slecht voorbeeld: Met oneindige stromen skipLast met een oneindige stroom
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Uitgang: 0(3(na een seconde), 1(4(na een seconde), 2(5(na een seconde), ...
// NDe uitvoer gaat oneindig door met een vertraging van 1
// De laatste3(na 1,5 seconde), met een vertraging van 1,5 seconde.
```

In het geval van oneindige streams blijven alle waarden worden uitgevoerd met een vertraging van N omdat de laatste N niet worden bepaald. Het oorspronkelijke doel van `skipLast` wordt niet bereikt, omdat er geen echte "laatste N" is.

**Oplossing**: `neem` naar een eindige stroom

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Goed voorbeeld: Na een eindige stroom skipLast met een oneindige stroom
interval(1000).pipe(
  take(10),      // Eerste logboekvermelding10Afgewerkt in 1 stuk
  skipLast(3)    // De laatste3Eén item overslaan
).subscribe(console.log);
// Uitgang: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wordt overgeslagen)
```

### 2. let op de buffergrootte

`skipLast(n)` houdt altijd n waarden in de buffer.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ 10001 stuk wordt bewaard in een buffer
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. Uitgangsvertraging.

`skipLast(n)` voert niets uit totdat n buffers zijn gevuld.

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('Invoer:', val)),
  skipLast(2)
).subscribe(val => console.log('Uitgang:', val));
// Invoer: 0
// Invoer: 1
// Invoer: 2
// Uitgang: 0  ← Uitvoer start wanneer buffer2Uitvoer start wanneer de buffer vol is
// Invoer: 3
// Uitgang: 1
// Invoer: 4
// Uitgang: 2
// Voltooiing (overslaan3, 4 (overslaan)
```

### 4. gedrag skipLast(0)

`skipLast(0)` slaat niets over.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0van (tot)9naar

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Uitgang: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wordt overgeslagen)
```

## 💡 Praktische combinatiepatronen

### Patroon 1: Alleen het tussenliggende deel krijgen

Sla het begin en einde over en krijg alleen het tussenstuk


### Patroon 2: Gegevensvalidatie

Als verificatie vereist is op volgende waarden


```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0van (tot)9naar

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Uitgang: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wordt overgeslagen)
```

### Patroon 3: Vensterverwerking

Vensterverwerking met gegevens exclusief de laatste N gevallen


## Gerelateerde operatoren

- **[skip](. /skip)** - eerste N waarden overslaan.
- **[takeLast](. /takeLast)** - alleen de laatste N waarden nemen.
- **[take](. /take)** - alleen de eerste N waarden ophalen.
- **[skipUntil](. /skipUntil)** - overslaan tot een andere Observable afgaat.
- **[skipWhile](. /skipWhile)** - overslaan terwijl aan voorwaarde is voldaan

## Samenvatting.

De `skipLast` operator slaat de laatste N waarden van de stream over.

- Ideaal als de laatste N gegevens niet nodig zijn.
- ✅ Nuttig voor het uitsluiten van onbepaalde gegevens
- ✅ Buffergrootte is slechts N (geheugenefficiënt)
- ✅ Voltooiing van stream vereist
- ⚠️ Niet beschikbaar voor oneindige streams
- ⚠️ Geen uitvoer totdat N buffers zijn opgebouwd
- ⚠️ Moet vaak gecombineerd worden met `take` voor eindige streams
