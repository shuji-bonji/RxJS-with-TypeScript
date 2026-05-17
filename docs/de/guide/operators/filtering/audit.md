---
description: "Der `audit`-Operator ist ein RxJS-Filteroperator, der nur den letzten Wert innerhalb eines durch einen benutzerdefinierten Observable gesteuerten Zeitraums emittiert. Ideal für dynamische Timing-Steuerung."
head:
  - - meta
    - name: keywords
      content: RxJS, audit, Filteroperator, dynamische Timing-Steuerung, Observable, TypeScript
---

# audit - Letzten Wert der Kontrollperiode emittieren

Der `audit`-Operator wartet, bis ein benutzerdefinierter Observable einen Wert emittiert, und emittiert den **letzten Wert**, den die Quelle in diesem Zeitraum emittiert hat.
Während `auditTime` durch eine feste Zeit gesteuert wird, ermöglicht `audit` die **dynamische Steuerung des Zeitraums** mit einem Observable.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Klick-Event
const clicks$ = fromEvent(document, 'click');

// 1Zeitraum jede Sekunde abgrenzen
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klick aufgezeichnet');
});
```

- Lorsqu'un clic se produit, une période d'une seconde commence.
- Seul le dernier clic de cette période d'une seconde est émis.
- Après une seconde, la période suivante commence.

[🌐 Documentation officielle de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet de désabonner `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/de/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Modèles d'utilisation typiques

- **Échantillonnage dynamique par intervalles** : ajuster la durée en fonction de la charge
- **Contrôle de timing personnalisé** : contrôle de la période basé sur d'autres Observables
- **Limitation adaptative des événements** : régulation sensible au contexte

## 🔍 Différences avec auditTime

| Operator | Periodensteuerung | Anwendungsfall |
|---|---|---|
| `auditTime` | Feste Zeit (Millisekunden) | Einfache zeitbasierte Steuerung |
| `audit` | **Benutzerdefinierte Observable** | **Dynamische Periodensteuerung** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fest1Sekunden
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fest1Sekunden'));

// audit - Dynamischer Zeitraum
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~2Zufälliger Zeitraum (Sekunden)
    return timer(period);
  })
).subscribe(() => console.log(`動的期間: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique basé sur la charge

Voici un exemple d'ajustement de l'intervalle d'échantillonnage en fonction de la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UIErstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Dynamisches Sampling</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Last ändern';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Lastniveau (0: Niedrig,1: Mittel,2: Hoch)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Niedrige Last', 'Mittlere Last', 'Hohe Last'];
  statusDiv.textContent = `Aktuelle Last: ${levels[loadLevel]}`;
});

// Mousemove-Event
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Zeitraum nach Last anpassen
    const periods = [2000, 1000, 500]; // Niedrige Last→Lange Periode, hohe Last→Kurze Periode
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Mausposition: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10Einträge anzeigen
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Échantillonnage réduit à des intervalles de 2 s lorsque la charge est faible (mode économie d'énergie)
- Échantillonnage fin à intervalles de 500 ms lorsque la charge est élevée
- La période peut être ajustée dynamiquement en fonction de la charge

## 🎯 Exemple de code pratique 2 : Contrôle de la période basé sur d'autres flux

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UIErstellen
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervall: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Slider-Wert überwachen
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Klick-Event
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Slider-Wert aktualisieren
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Klickauditgesteuert durch
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Klick-Aufzeichnung (Intervall: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Points d'attention

### 1. La première valeur n'est pas émise immédiatement

`audit` attend la fin de la période après avoir reçu la première valeur.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Ausgabe:
// 9  (1Sekunden später,0~9letzter Wert von)
// 19 (2Sekunden später,10~19letzter Wert von)
// 29 (3Sekunden später,20~29letzter Wert von)
```

### 2. L'Observable de durée est généré à chaque fois

La fonction passée à `audit` **doit retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Schlechtes Beispiel: GleicheObservableInstanz wird wiederverwendet
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2funktioniert nach dem 2. Mal nicht mehr
).subscribe();

// ✅ Gutes Beispiel: Jedes Mal neuObservableGeneriert
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Mémoire et performance

L'utilisation de `audit` sur des flux où des valeurs sont fréquemment émises consomme de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Schneller Stream (10ms jede)
interval(10).pipe(
  audit(() => timer(1000)) // 1Sampling jede Sekunde
).subscribe();
// 1In N Sekunden,100Werte sammeln sich im Speicher an und nur der letzte1wird emittiert
```

## 🆚 Comparaison avec des opérateurs similaires

| Operator | Wann emittiert | Emittierter Wert | Anwendungsfall |
|---|---|---|---|
| `audit` | Am **Ende** der Periode | Der **letzte** Wert der Periode | Letzten Zustand in der Periode abrufen |
| `throttle` | Am **Anfang** der Periode | Der **erste** Wert der Periode | Anfang einer Ereignisfolge erhalten |
| `debounce` | **Nach Stabilisierung** | Wert kurz vor Stabilisierung | Auf Eingabeabschluss warten |
| `sample` | **Wenn ein anderer Observable auslöst** | Aktuellster Wert zu diesem Zeitpunkt | Periodischer Snapshot |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Letzter Klick in N Sekunden
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Letzte'));

// throttle: 1Erster Klick in N Sekunden
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Erste'));

// debounce: Nach Klick-Stopp1Sekunden später
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Nach Stopp'));

// sample: 1Sampling jede Sekunde
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periodisch'));
```

## 📚 Opérateurs associés

- **[auditTime](./auditTime)** - contrôlé par un temps fixe (version simplifiée de `audit`)
- **[throttle](./throttleTime)** - première valeur émise au début de la période
- **[debounce](./debounceTime)** - émission d'une valeur après stabilisation
- **[sample](./sampleTime)** - échantillonnage au moment d'un autre Observable

## Résumé

L'opérateur `audit` émet la dernière valeur d'une période dynamiquement contrôlée par un Observable personnalisé.

- ✅ Contrôle dynamique de la période possible
- ✅ Échantillonnage adaptatif basé sur la charge
- ✅ Contrôle basé sur d'autres flux
- ⚠️ Un nouvel Observable doit être généré à chaque fois
- ⚠️ Attention à la mémoire en cas d'émission fréquente
