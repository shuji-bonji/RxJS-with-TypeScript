---
description: bufferCount is een RxJS conversieoperator die een array van waarden uitvoert voor elk gespecificeerd aantal items. Het is ideaal voor batchverwerking, data-aggregatie op vast aantal, pakketsplitsing en andere aantal-gebaseerde streamcontrole, en maakt type-veilige arrayoperaties mogelijk door TypeScript's type-inferentie.
---

# bufferCount - Verzamel waarden op gespecificeerd aantal

De `bufferCount` operator **groepeert** een gespecificeerd aantal uitgezonden waarden samen en voert ze uit als een array.
Dit is nuttig voor batchverwerking waar u waarden wilt scheiden op aantal.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// Geef waarden elke 100ms uit
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('Waarden elke 5:', buffer);
});

// Output:
// Waarden elke 5: [0, 1, 2, 3, 4]
// Waarden elke 5: [5, 6, 7, 8, 9]
// ...
```

- Voert een array van 5 waarden per keer uit.
- Het is uniek doordat het op een **aantal basis** groepeert, niet op een tijdbasis.

[🌐 RxJS Officiële Documentatie - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 Typische gebruikspatronen

- Splits en verzend datapakketten
- Batch opslaan of batchverwerking op een bepaald aantal
- Aggregatie van invoergebeurtenissen op een bepaald aantal voorkomens

## 🧠 Praktisch codevoorbeeld (met UI)

Dit is een voorbeeld van het weergeven van een samenvatting van toetsenbordaanslagen elke 5 aanslagen.

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Toetsinvoer gebeurtenisstream
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5 invoer: ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- Elke keer dat een toets vijf keer wordt ingedrukt, worden die vijf aanslagen samen weergegeven.
- U kunt het aggregatieproces op basis van het aantal ervaren.
