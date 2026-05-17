---
description: "La fonction de création merge s'abonne à plusieurs Observables simultanément et fusionne les valeurs en temps réel : Essentielle pour l'intégration de flux d'événements parallèles"
---

# merge - fusionner plusieurs flux simultanément

`merge` est une fonction de création qui s'abonne à plusieurs Observables simultanément et qui produit les valeurs au fur et à mesure qu'elles sont émises par chaque Observable.

## Syntaxe de base et utilisation

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Flux 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Flux 2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// Exemple de sortie:
// Flux 1: 0
// Flux 2: 0
// Flux 1: 1
// Flux 1: 2
// Flux 2: 1
```

- S'abonner à tous les Observables simultanément, et les valeurs circuleront dans **l'ordre d'émission**.
- Il n'y a pas de garantie d'ordre, et cela **dépend** du moment où chaque Observable émet.


[🌐 Documentation officielle RxJS - `merge`](https://rxjs.dev/api/index/function/merge)

## Modèles d'utilisation typiques

- **Fusionner** plusieurs événements asynchrones (par exemple, les entrées de l'utilisateur et les notifications du backend)
- **Regrouper plusieurs flux de données en un seul flux**
- **Combiner des sources d'information parallèles**, par exemple des mises à jour en temps réel et l'intégration de polling

## Exemples de code pratique (avec interface utilisateur)

Combine les événements de clic et de minuterie en temps réel.

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs';

// Créer une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de merge:</h3>';
document.body.appendChild(output);

// Créer un élément bouton
const button = document.createElement('button');
button.textContent = 'Cliquer pour déclencher un événement';
document.body.appendChild(button);

// Flux de clic
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ Clic sur le bouton détecté')
);

// Flux de minuterie
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ Événement minuterie (${val})`)
);

// Fusionner et afficher
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **Cliquez sur un bouton et un événement est généré** immédiatement,
- **La minuterie déclenche un événement répétitif** toutes les 3 secondes.
- Vous pouvez voir comment deux types différents d'Observables peuvent être fusionnés en **temps réel**.

> [!WARNING] Attention en code de production
> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)


## Opérateurs associés

- **[mergeWith](/fr/guide/operators/combination/mergeWith)** - Version opérateur Pipeable (utilisée dans le pipeline)
- **[mergeMap](/fr/guide/operators/transformation/mergeMap)** - mappe et concatène chaque valeur en parallèle
- **[concat](/fr/guide/creation-functions/combination/concat)** - Fonction de création de concaténation séquentielle
