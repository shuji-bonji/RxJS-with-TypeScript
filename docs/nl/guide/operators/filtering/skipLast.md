---
description: "skipLast operator slaat de laatste N waarden in Observable-streams over en geeft alleen eerdere waarden uit: Perfect voor het uitsluiten van onbevestigde wachtende data"
---

# skipLast - Sla laatste N waarden over

De `skipLast` operator **slaat de laatste N waarden over** die door de bron Observable worden uitgegeven en geeft alleen de waarden ervoor uit. Het houdt de laatste N waarden in een buffer tot de stream voltooit en geeft de rest uit.

## 🔰 Basissyntax en gebruik

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 tot 9

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 worden overgeslagen)
```

**Werkingsstroom**:
1. Stream geeft 0, 1, 2, ... uit
2. Houdt laatste 3 waarden (7, 8, 9) in buffer
3. Geeft waarden uit die buffergrootte overschrijden (0~6)
4. Wanneer stream voltooit, worden bufferwaarden (7, 8, 9) weggegooid zonder uitvoer

[🌐 RxJS Officiële Documentatie - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Typische gebruikspatronen

- **Laatste data uitsluiten**: Onbevestigde laatste data uitsluiten
- **Batchverwerking**: Wachtende data uitsluiten voordat verwerking voltooit
- **Datavalidatie**: Wanneer validatie vereist is op volgende waarden
- **Vertraagde gefinaliseerde dataverwerking**: Wanneer de laatste N items niet gefinaliseerd zijn

## 🧠 Praktisch codevoorbeeld 1: Dataverwerkingspipeline

Voorbeeld van het overslaan van de laatste wachtende data in dataverwerking.

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// Maak UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Dataverwerkingspipeline';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = 'Sla laatste 2 items over (wachtende data) voor verwerking';
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

// Datastream (10 items)
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// Geef data elke 0,5 seconden uit
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // Sla laatste 2 items over
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ Bevestigd</strong>
      ID: ${item.id} |
      Waarde: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // Toon overgeslagen items
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ Overgeslagen</strong>
        ID: ${item.id} |
        Waarde: ${item.value} |
        (Wachtende data)
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `Verwerking voltooid: ${data.length - 2} bevestigd, 2 overgeslagen`;
    output.appendChild(summary);
  }
});
```

- Data wordt sequentieel verwerkt, maar de laatste 2 items worden behandeld als wachtend en overgeslagen.
- Na voltooiing worden de overgeslagen items ook weergegeven.

## 🆚 Vergelijking met vergelijkbare operators

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0 tot 9

// skipLast: Sla laatste N waarden over
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6

// takeLast: Neem alleen laatste N waarden
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9

// skip: Sla eerste N waarden over
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, 8, 9
```

| Operator | Overslaanpositie | Uitvoertiming | Vereist voltooiing |
|:---|:---|:---|:---|
| `skipLast(n)` | Laatste n waarden | Uitvoer wanneer buffer overschreden | Vereist |
| `takeLast(n)` | Alles behalve laatste n | Alles samen uitvoeren na voltooiing | Vereist |
| `skip(n)` | Eerste n waarden | Onmiddellijk uitvoeren | Niet vereist |

## ⚠️ Belangrijke opmerkingen

### 1. Gedrag met oneindige streams

Omdat `skipLast` de laatste N waarden niet kan identificeren tot voltooiing, werkt het niet zoals bedoeld met oneindige streams.

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Slecht voorbeeld: Gebruik skipLast met oneindige stream
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0 (na 3 seconden), 1 (na 4 seconden), 2 (na 5 seconden), ...
// Alle waarden blijven oneindig uitvoeren met N vertraging
// Laatste 3 blijven voor altijd in buffer en worden nooit uitgevoerd
```

**Oplossing**: Maak het een eindige stream met `take`

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Goed voorbeeld: Maak eindige stream voor gebruik van skipLast
interval(1000).pipe(
  take(10),      // Voltooi met eerste 10 waarden
  skipLast(3)    // Sla laatste 3 over
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 worden overgeslagen)
```

### 2. Let op buffergrootte

`skipLast(n)` houdt altijd n waarden in de buffer.

### 3. Uitvoervertraging

`skipLast(n)` geeft niets uit tot n buffers zijn gevuld.

### 4. skipLast(0) gedrag

`skipLast(0)` slaat niets over.

## 📚 Gerelateerde operators

- **[skip](/nl/guide/operators/filtering/skip)** - Sla eerste N waarden over
- **[takeLast](/nl/guide/operators/filtering/takeLast)** - Neem alleen laatste N waarden
- **[take](/nl/guide/operators/filtering/take)** - Neem alleen eerste N waarden
- **[skipUntil](/nl/guide/operators/filtering/skipUntil)** - Sla over tot andere Observable vuurt
- **[skipWhile](/nl/guide/operators/filtering/skipWhile)** - Sla over terwijl voorwaarde voldaan is

## Samenvatting

De `skipLast` operator slaat de laatste N waarden in de stream over.

- ✅ Ideaal wanneer de laatste N data niet nodig zijn
- ✅ Nuttig voor het uitsluiten van onbevestigde data
- ✅ Buffergrootte is alleen N (geheugenefficiënt)
- ✅ Vereist streamvoltooiing
- ⚠️ Kan niet gebruiken met oneindige streams
- ⚠️ Geen uitvoer tot buffer gevuld is met N waarden
- ⚠️ Moet vaak worden gecombineerd met `take` om eindige stream te maken
