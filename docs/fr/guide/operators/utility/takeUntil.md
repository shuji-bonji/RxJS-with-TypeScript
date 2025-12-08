---
description: "L'opérateur takeUntil est utilisé pour s'abonner à l'Observable source jusqu'à ce que l'Observable notificateur émette une valeur, à partir de laquelle l'abonnement est désabonné."
---

# takeUntil

`takeUntil` est un opérateur qui **continue à s'abonner à l'Observable source jusqu'à ce que l'Observable spécifié (déclencheur de notification) émette sa première valeur**. Lorsque le déclencheur de notification émet, l'abonnement à l'Observable source est désabonné.

## 🔁 Syntaxe de base

```ts
source$.pipe(
  takeUntil(notifier$)
)
```

- `source$` : L'Observable source (cible de l'abonnement)
- `notifier$` : L'Observable qui signale l'arrêt (lorsque cet Observable émet sa première valeur, l'abonnement s'arrête)

[🌐 Documentation officielle RxJS - takeUntil](https://rxjs.dev/api/index/function/takeUntil)

## 🧪 Exemple d'utilisation : Arrêter l'abonnement au clic d'un bouton

```ts
import { interval, fromEvent } from 'rxjs';
import { takeUntil } from 'rxjs';

const stopButton = document.createElement('button');
stopButton.textContent = 'stop';
document.body.appendChild(stopButton)

const stop$ = fromEvent(stopButton, 'click');
const source$ = interval(1000); // Émet des nombres toutes les secondes

source$
  .pipe(takeUntil(stop$))
  .subscribe((val) => console.log(`Valeur : ${val}`));
```

📌 Lorsque `stopButton` est cliqué, l'abonnement à `source$` est arrêté à ce moment.

## ✅ Cas d'utilisation courants

- Lorsque vous souhaitez arrêter les requêtes HTTP ou le polling à l'aide d'un bouton d'annulation
- Lorsque vous souhaitez vous désabonner en fonction du cycle de vie d'un composant
- Lorsque vous souhaitez interrompre un traitement asynchrone sur des transitions de page ou un démontage

## 🔗 Opérateurs associés

- `take` : Reçoit des valeurs jusqu'à un certain nombre de fois
- `first` : N'obtient que le premier élément et termine
- `skipUntil` : Ignore les valeurs jusqu'à ce qu'un Observable spécifique soit émis
