---
description: "L'opérateur map est un outil de transformation de base qui applique une fonction à chaque valeur d'un Observable pour générer de nouvelles valeurs. Couramment utilisé pour la mise en forme de formulaires, le traitement des réponses API et la transformation de données. Explique l'inférence de type TypeScript, la combinaison avec d'autres opérateurs et l'optimisation des performances."
---

# map - Appliquer une fonction de transformation à chaque valeur

L'opérateur `map` applique une fonction spécifiée à **chaque valeur** dans un flux et génère de nouvelles valeurs transformées.
Il est similaire à la méthode `Array.prototype.map`, mais opère sur des **flux asynchrones**.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// Sortie : 10, 20, 30
```

La fonction `value => value * 10` est appliquée à chaque valeur, générant de nouvelles valeurs.

[🌐 Documentation officielle RxJS - map](https://rxjs.dev/api/index/function/map)

## 💡 Modèles d'utilisation typiques

- Transformation des réponses API (extraction des seules propriétés nécessaires)
- Formatage des données saisies dans les formulaires
- Traitement de nombres ou de chaînes de caractères dans des flux
- Extraire uniquement les données nécessaires des événements de l'interface utilisateur

## 🧠 Exemple de code pratique (avec interface utilisateur)

Exemple qui double les nombres saisis en temps réel.

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Création du champ de saisie
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Entrez un nombre';
document.body.appendChild(input);

// Création du champ de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Flux d'événements de saisie
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `Valeur doublée : ${result}`;
});
```

- La valeur d'entrée est doublée en temps réel pour la sortie.
- En enchaînant les opérateurs map, on obtient une chaîne de transformation de données simple.
