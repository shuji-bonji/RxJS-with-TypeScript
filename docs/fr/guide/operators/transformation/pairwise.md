---
description: "L'opérateur pairwise produit deux valeurs consécutives sous la forme d'un tableau de paires [valeur précédente, valeur actuelle]. Utile pour la détection des changements, le calcul des différences, l'analyse des tendances, l'interpolation des animations et la comparaison des valeurs précédentes et actuelles. Explique l'implémentation TypeScript sécurisée."
---

# pairwise - Traiter deux valeurs consécutives par paires

L'opérateur `pairwise` **produit deux valeurs consécutives d'un flux sous la forme d'un tableau [valeur précédente, valeur actuelle]**.
Pratique pour comparer les valeurs précédentes et actuelles ou calculer la quantité de changement.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Sortie :
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- La première valeur (0) n'est pas émise seule ; lorsque la deuxième valeur (1) arrive, elle est émise sous la forme `[0, 1]`.
- Produit toujours une paire de **la valeur précédente et la valeur actuelle**.

[🌐 Documentation officielle RxJS - pairwise](https://rxjs.dev/api/operators/pairwise)

## 💡 Modes d'utilisation typiques

- Calcul de l'amplitude du mouvement de la souris ou du toucher
- Calcul du montant de variation (différence) des prix ou nombres
- Détection des changements d'état (comparaison de l'état précédent et actuel)
- Détermination de la direction de défilement

## 🧠 Exemple de code pratique (avec interface utilisateur)

Exemple affichant la direction et l'amplitude du mouvement de la souris.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Événement de mouvement de la souris
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'droite' : deltaX < 0 ? 'gauche' : 'arrêt';

  output.innerHTML = `
    Précédent : (${prev.x}, ${prev.y})<br>
    Actuel : (${curr.x}, ${curr.y})<br>
    Déplacement : Δx=${deltaX}, Δy=${deltaY}<br>
    Direction : ${direction}
  `;
});
```

- Lorsque la souris est déplacée, les coordonnées précédentes et actuelles ainsi que l'amplitude du mouvement sont affichées.
- Avec `pairwise`, les coordonnées précédentes et actuelles sont automatiquement obtenues par paire.

## 🎯 Calcul du montant de variation des nombres

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (nombres carrés)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (différence : +${result.diff})`);
});

// Sortie :
// 0 → 1 (différence : +1)
// 1 → 4 (différence : +3)
// 4 → 9 (différence : +5)
// 9 → 16 (différence : +7)
// 16 → 25 (différence : +9)
```

## 🔍 Comparaison avec bufferCount(2, 1)

`pairwise()` a le même comportement que `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Sortie : [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Sortie : [1,2], [2,3], [3,4], [4,5]
```

**Utilisation** :
- `pairwise()` : Explicite pour traiter des paires de deux valeurs consécutives, intention claire du code
- `bufferCount(2, 1)` : Plus flexible (peut gérer des tailles de fenêtre supérieures à 3)

## ⚠️ Notes importantes

### La première valeur n'est pas émise

`pairwise` n'émet rien tant que deux valeurs ne sont pas prêtes, donc la première valeur ne peut pas être obtenue seule.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('Terminé')
});

// Sortie :
// Terminé
// (aucune valeur n'est émise)
```

**Solution** : Pour traiter la première valeur, ajoutez une valeur initiale avec `startWith`.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Sortie :
// [0, 10]
// [10, 20]
// [20, 30]
```

## 📚 Opérateurs associés

- [`scan`](./scan) - Traitement d'accumulation plus complexe
- [`bufferCount`](./bufferCount) - Regrouper des valeurs par nombre spécifié
- [`distinctUntilChanged`](../filtering/distinctUntilChanged) - Supprimer les valeurs dupliquées consécutives
- [`startWith`](/fr/guide/operators/utility/startWith) - Ajouter une valeur initiale

## Résumé

L'opérateur `pairwise` produit deux valeurs consécutives sous forme de paire `[valeur précédente, valeur actuelle]`. Très utile pour **les situations nécessitant une comparaison entre les valeurs précédentes et actuelles**, comme le suivi du mouvement de la souris, le calcul des variations de prix et la détection des transitions d'état.
