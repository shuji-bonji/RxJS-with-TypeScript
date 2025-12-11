---
description: takeLast is een RxJS filteroperator die alleen de laatste N waarden uitvoert wanneer de Observable-stream voltooit. Het is ideaal voor scenario's waar alleen de laatste waarden van de hele stream nodig zijn, zoals het ophalen van de nieuwste logvermeldingen, het weergeven van top N items op een ranglijst, en einddata-samenvattingen bij voltooiing. Het kan niet worden gebruikt met oneindige streams omdat het waarden in een buffer vasthoudt tot voltooiing.
---

# takeLast - Haal laatste N waarden op

De `takeLast` operator voert alleen de laatste N waarden uit wanneer de stream **voltooit**. Het houdt waarden in een buffer vast tot de stream voltooit, en voert ze dan allemaal tegelijk uit.


## 🔰 Basissyntax en gebruik

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 tot 9

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

**Werkingsstroom**:
1. Stream geeft 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 uit
2. Houdt intern laatste 3 waarden in buffer vast
3. Stream voltooit
4. Voert bufferwaarden 7, 8, 9 op volgorde uit

[🌐 RxJS Officiële Documentatie - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 Contrast met take

`take` en `takeLast` hebben contrasterend gedrag.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 tot 9

// take: Haal eerste N waarden op
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2 (onmiddellijk uitvoer)

// takeLast: Haal laatste N waarden op
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9 (uitvoer na wachten op voltooiing)
```

| Operator | Ophaal positie | Uitvoertiming | Gedrag voor voltooiing |
|---|---|---|---|
| `take(n)` | Eerste n waarden | Onmiddellijk uitvoer | Auto-voltooien na n waarden |
| `takeLast(n)` | Laatste n waarden | Allemaal samen uitvoeren na voltooiing | Vasthouden in buffer |


## 💡 Typische gebruikspatronen

1. **Haal nieuwste N logvermeldingen op**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App gestart' },
     { timestamp: 2, level: 'info' as const, message: 'Gebruiker ingelogd' },
     { timestamp: 3, level: 'warn' as const, message: 'Trage query gedetecteerd' },
     { timestamp: 4, level: 'error' as const, message: 'Verbinding mislukt' },
     { timestamp: 5, level: 'info' as const, message: 'Nieuwe poging succesvol' },
   ] as LogEntry[]);

   // Haal nieuwste 3 logvermeldingen op
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Output:
   // [warn] Trage query gedetecteerd
   // [error] Verbinding mislukt
   // [info] Nieuwe poging succesvol
   ```

2. **Haal top N op ranglijst op**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]);

   // Haal top 3 op
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Output: Charlie: 200, Dave: 180, Eve: 220
   ```


## ⚠️ Belangrijke opmerkingen

> [!WARNING]
> `takeLast` **wacht tot de stream voltooit**, dus het werkt niet met oneindige streams. Ook, als n in `takeLast(n)` groot is, verbruikt het veel geheugen.

### 1. Kan niet gebruiken met oneindige streams

`takeLast` wacht tot de stream voltooit, dus het werkt niet met oneindige streams.

```ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Slecht voorbeeld: Gebruik takeLast met oneindige stream
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// Niets uitgevoerd (omdat stream nooit voltooit)
```

**Oplossing**: Maak het een eindige stream door te combineren met `take`

```ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Goed voorbeeld: Gebruik takeLast na het maken van eindige stream
interval(1000).pipe(
  take(10),      // Voltooi met eerste 10 waarden
  takeLast(3)    // Haal laatste 3 daarvan op
).subscribe(console.log);
// Output: 7, 8, 9
```

### 2. Let op geheugengebruik

`takeLast(n)` houdt de laatste n waarden in een buffer vast, dus grote n verbruikt geheugen.

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Let op: Houd grote hoeveelheid data in buffer vast
range(0, 1000000).pipe(
  takeLast(100000) // Houd 100.000 items in geheugen vast
).subscribe(console.log);
```


## 🎯 Verschil met last

```ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: Alleen laatste 1 waarde
numbers$.pipe(
  last()
).subscribe(console.log);
// Output: 9

// takeLast(1): Laatste 1 waarde (uitvoer als enkele waarde, niet array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// Output: 9

// takeLast(3): Laatste 3 waarden
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

| Operator | Ophaal aantal | Voorwaarde specificatie | Gebruiksscenario |
|---|---|---|---|
| `last()` | 1 waarde | Mogelijk | Laatste 1 waarde of laatste 1 waarde die aan voorwaarde voldoet |
| `takeLast(n)` | n waarden | Niet mogelijk | Simpelweg laatste n waarden ophalen |


## 🎓 Samenvatting

### Wanneer takeLast gebruiken
- ✅ Wanneer u de laatste N data van de stream nodig heeft
- ✅ Wanneer u nieuwste N entries van logs of transacties wilt ophalen
- ✅ Wanneer streamvoltooiing gegarandeerd is
- ✅ Wanneer u data-samenvatting of top N items wilt weergeven

### Wanneer take gebruiken
- ✅ Wanneer u de eerste N data van de stream nodig heeft
- ✅ Wanneer u onmiddellijk resultaten wilt ophalen
- ✅ Wanneer u een deel van een oneindige stream wilt ophalen

### Opmerkingen
- ⚠️ Kan niet gebruiken met oneindige streams (voltooit niet)
- ⚠️ Grote n in `takeLast(n)` verbruikt geheugen
- ⚠️ Uitvoer gebeurt allemaal samen na voltooiing (niet onmiddellijk uitvoer)
- ⚠️ Moet vaak worden gecombineerd met `take(n)` om eindige stream te maken


## 🚀 Volgende stappen

- **[take](/nl/guide/operators/filtering/take)** - Leer hoe u eerste N waarden ophaalt
- **[last](/nl/guide/operators/filtering/last)** - Leer hoe u laatste 1 waarde ophaalt
- **[skip](/nl/guide/operators/filtering/skip)** - Leer hoe u eerste N waarden overslaat
- **[filter](/nl/guide/operators/filtering/filter)** - Leer hoe u filtert op basis van voorwaarden
- **[Filteroperator praktische voorbeelden](/nl/guide/operators/filtering/practical-use-cases)** - Leer echte use cases
