---
description: "El operador `audit` es un operador de filtrado RxJS que emite solo el último valor dentro del periodo controlado por un Observable personalizado. Ideal para el control dinámico de tiempo."
head:
  - - meta
    - name: keywords
      content: RxJS, audit, operador de filtrado, control dinámico de tiempo, Observable, TypeScript
---

# audit - dernière valeur de la période de contrôle émise

L'opérateur `audit` attend qu'un Observable personnalisé émette une valeur et émet la **dernière valeur** émise par la source au cours de cette période.
Alors que `auditTime` est contrôlé par un temps fixe, `audit` permet de **contrôler la période** avec un Observable dynamique.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento clic
const clicks$ = fromEvent(document, 'click');

// 1Delimitar el periodo cada N segundos
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Clic registrado');
});
```

- Lorsqu'un clic se produit, une période d'une seconde commence.
- Seul le dernier clic de cette période d'une seconde est émis.
- Après une seconde, la période suivante commence.

[🌐 Documentation officielle de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet de désabonner `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/es/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Modèles d'utilisation typiques

- **Échantillonnage dynamique par intervalles** : ajuster la durée en fonction de la charge
- **Contrôle de timing personnalisé** : contrôle de la période basé sur d'autres Observables
- **Limitation adaptative des événements** : régulation sensible au contexte

## 🔍 Différences avec auditTime

| Operador | Control del periodo | Caso de uso |
|---|---|---|
| `auditTime` | Tiempo fijo (milisegundos) | Control simple basado en tiempo |
| `audit` | **Observable personalizado** | **Control dinámico del periodo** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fijo1segundos
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fijo1segundos'));

// audit - Periodo dinámico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~2Periodo aleatorio de segundos
    return timer(period);
  })
).subscribe(() => console.log(`動的期間: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique basé sur la charge

Voici un exemple d'ajustement de l'intervalle d'échantillonnage en fonction de la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICrear
const output = document.createElement('div');
output.innerHTML = '<h3>Muestreo dinámico</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Cambiar carga';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Nivel de carga (0: Baja,1: Media,2: Alta)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carga baja', 'Carga media', 'Carga alta'];
  statusDiv.textContent = `Carga actual: ${levels[loadLevel]}`;
});

// Evento mousemove
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Ajustar periodo según carga
    const periods = [2000, 1000, 500]; // Carga baja→Periodo largo, carga alta→Periodo corto
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posición del ratón: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Máx.10elementos mostrados
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

// UICrear
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervalo: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Monitorizar valor del slider
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Evento clic
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Actualizar valor del slider
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Clicauditcontrolado por
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Registro de clic (intervalo: ${currentInterval}ms)`;
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
// Salida:
// 9  (1segundos después,0~9último valor de)
// 19 (2segundos después,10~19último valor de)
// 29 (3segundos después,20~29último valor de)
```

### 2. L'Observable de durée est généré à chaque fois

La fonction passée à `audit` **doit retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Mal ejemplo: MismaObservableinstancia reutilizada
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2no funciona tras la 2a vez
).subscribe();

// ✅ Buen ejemplo: Nuevo cada vezObservableGenera
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Mémoire et performance

L'utilisation de `audit` sur des flux où des valeurs sont fréquemment émises consomme de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Stream rápido (10ms cada)
interval(10).pipe(
  audit(() => timer(1000)) // 1Muestreo cada segundo
).subscribe();
// 1En N segundos,100valores se acumulan en memoria y solo el último1es emitido
```

## 🆚 Comparaison avec des opérateurs similaires

| Operador | Cuándo emitir | Valor emitido | Caso de uso |
|---|---|---|---|
| `audit` | Al **final** del periodo | El **último** valor del periodo | Obtener el último estado del periodo |
| `throttle` | Al **inicio** del periodo | El **primer** valor del periodo | Obtener el inicio de una secuencia de eventos |
| `debounce` | **Tras estabilización** | Valor justo antes de estabilización | Esperar finalización de entrada |
| `sample` | **Cuando otro Observable se dispara** | Valor más reciente en ese momento | Instantánea periódica |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Último clic en N segundos
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Último'));

// throttle: 1Primer clic en N segundos
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Primero'));

// debounce: Tras detener clic1segundos después
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Tras detener'));

// sample: 1Muestreo cada segundo
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periódico'));
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
